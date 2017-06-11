#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <ctime>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <angles/angles.h>

#include "drive.h"
#include "PID.h"
#include "rpd.h"
#include "cruisecontrol.h"

#define PUB_TO_RVIZ

#define PLUS 0x2B
#define MINUS 0x2D

/* see turtlebot specs  */
#define TURTLEBOT_VEL_MAX_MPS .65
#define TURTLEBOT_ANG_MAX_RPS  M_PI

using namespace tf;
using namespace geometry_msgs;
using namespace angles;

/* boolean flags for callbacks and "drive()" */
static bool reached_goal = true;
static bool odom_ok  = false;
static bool map_pose_ok = false;
static bool drive_path = true;

static double p = 0, i = 0, d = 0, theta = 0,  eps = .01;
static double dt = 0;

/* parameters for reading keyboard input */
static struct termios old_tio, new_tio;

static void setup_IO(){

    /* get the terminal settings for stdin */
    tcgetattr(STDIN_FILENO,&old_tio);

    /* we want to keep the old setting to restore them a the end */
    new_tio=old_tio;

    /* disable canonical mode (buffered i/o) and local echo */
    new_tio.c_lflag &=(~ICANON & ~ECHO);

    /* set the new settings immediately */
    tcsetattr(STDIN_FILENO,TCSANOW,&new_tio);

}

static void reset_IO(){
    /* restore the former settings */
    tcsetattr(STDIN_FILENO,TCSANOW,&old_tio);

}

Drive::Drive(ros::NodeHandle &nh, const std::string &pose, const std::string &goals):nh_(nh),pose_(),listener_(){

    /* member function as callback */
    nav_goals_sub_ = nh_.subscribe(goals,1,&Drive::nav_goals_callback,this);
    pose_sub_ = nh_.subscribe(pose,1,&Drive::pose_callback,this);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",10);
    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("drive/marker",10);

    /* dispatch keyboard thread */
    setup_IO();
    run_thread_ = true;
    thread = new boost::thread(boost::bind(&Drive::keyboard_thread, this));

    ROS_INFO_STREAM("ANGLE MIN: " << ANGLE_MIN_RPS << " ANGLE MAX: " << ANGLE_MAX_RPS);
}

void Drive::nav_goals_callback(const nav_msgs::Path::ConstPtr &nav_msg){

    pose_.clear();

    ROS_INFO_STREAM("NEW NAV_GOALS RECEIVED");

    for (auto i = 0; i < nav_msg->poses.size(); i++){
        pose_.push_back(nav_msg->poses[i]);
    }

    /* assign new goals */
    reached_goal = true;
}

void Drive::pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose){
    static auto last_cb_time = std::clock();
    auto time = std::clock();
    dt = double(time - last_cb_time)/CLOCKS_PER_SEC;;
    last_cb_time = time;
    ROS_INFO_STREAM("POSE RECEIVED");

    /* assign map_pose */
    //memcpy(&amcl_pose_.pose,&pose->pose.pose,sizeof(geometry_msgs::Pose));
    amcl_pose_.header = pose->header;
    amcl_pose_.pose.pose.position = pose->pose.pose.position;
    amcl_pose_.pose.pose.orientation = pose->pose.pose.orientation;
    //memcpy(&amcl_pose_.pose.pose.position,&pose->pose.pose.position,sizeof(tf::Pose));
    ROS_INFO_STREAM("CURRENT POSE x: " << amcl_pose_.pose.pose.position.x << " y: " << amcl_pose_.pose.pose.position.y << " theta: " << to_degrees(getYaw(amcl_pose_.pose.pose.orientation)));
    map_pose_ok = true;
}

static void sleep_ms(int ms){
    boost::this_thread::sleep_for(boost::chrono::milliseconds{ms});
}

void Drive::keyboard_thread(){

    ROS_INFO_STREAM("STARTING KEYBOARD THREAD");


    ROS_INFO_STREAM("PRESS \"a\", \"s\", \"d\" to increase P I D values");
    ROS_INFO_STREAM("PRESS \"y\", \"x\", \"c\" to decrease P I D values");
    ROS_INFO_STREAM("PRESS \"f\", \"v\" to in-/decrease min/max angle values in DEGREE");
    ROS_INFO_STREAM("PRESS +/- to in/decrease epsilon");

    static double p = P_START, i = I_START, d = D_START, theta = ANGLE_MAX_RPS*180./M_PI;
    double eps_ = eps;

    while(run_thread_){

        double p_ = p, i_ = i, d_ = d, theta_ = theta;

        unsigned char c = getchar();


        if (c == 'a') p_ += .1;
        else if (c == 's') i_ += .1;
        else if (c == 'd') d_ += .1;
        else if (c == 'y') p_ -= .1;
        else if (c == 'x') i_ -= .1;
        else if (c == 'c') d_ -= .1;
        else if (c == 'f') theta_ += 5.;
        else if (c == 'v') theta_ -= 5.;
        else if (c == PLUS) eps_ += .01;
        else if (c == MINUS) eps_ -= .01;


        if (p_ < 0) p_ = 0;
        if (i_ < 0) i_ = 0;
        if (d_ < 0) d_ = 0;
        if (theta_ < 5.) theta_ = 5;
        if(eps_ <= 0) eps_ = .01;

        if (p_ != p || i_ != i || d_ != d || theta_ != theta || eps_ != eps){
            ROS_INFO_STREAM("P: " << p_ << " I: " << i_ << " D: " << d_ << " THETA: +/- " << theta_ << " EPS:" << eps_);
        }

        mtx_.lock();
        //angle_ctrl.set_new_params(p_,i_,d_,-M_PI/180.*theta, M_PI/180.*theta);
        eps = eps_;
        mtx_.unlock();

        p = p_;
        i = i_;
        d = d_;
        theta = theta_;

        sleep_ms(200);
    }
    thread->join();
}


void Drive::PublishCmdVel(double angular, double speed){

    geometry_msgs::Twist msg;

    msg.angular.z = angular;
    msg.linear.x = speed;
    twist_pub_.publish(msg);

}

void Drive::PublishMarker(const std::vector<Vector2D> &path_approx){

    static visualization_msgs::MarkerArray Markers;
    rviz_visual_tools::RvizVisualTools tools("map");

    Markers.markers.clear();

    tf::Quaternion q = tf::createQuaternionFromRPY(0,0,0);

    int id = 0;

    /* markers to publish */
    for (auto v : path_approx){

        visualization_msgs::Marker marker;

        marker.header.frame_id="map";
        marker.header.stamp = ros::Time();
        marker.id = id++;
        marker.ns = "drive";
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = v.getX();
        marker.pose.position.y = v.getY();
        marker.pose.position.z = .25;
        marker.scale.x = .1;
        marker.scale.y = .1;
        marker.scale.z = .5;
        memcpy(&marker.pose.orientation,&q,sizeof(tf::Quaternion));
        marker.color.b = 0;
        marker.color.g = 0;
        marker.color.r = 1;
        marker.color.a = 1;

        Markers.markers.push_back(marker);

    }

    /* delete all markers */
    tools.deleteAllMarkers();

    /* publish new markers */
    marker_pub_.publish(Markers);

}

bool Drive::convertToRobotFrame(const PoseWithCovarianceStamped &p, pose_t &out){

    static tf::TransformListener listener;
    tf::StampedTransform transform_in_map;

    try{

        // bool got_transform = listener.waitForTransform("base_foot_print","map",ros::Time(0),ros::Duration(3.0));
        listener.lookupTransform("map", "base_footprint", ros::Time(0), transform_in_map);

        /* no transform avaiable */
        // if(!got_transform) return false;

        /* transform global coordinates to base_link */
        geometry_msgs::PoseStamped current,trans;
        current.header.frame_id="map";
        current.header.stamp= ros::Time(0);
        memcpy(&current.pose,&p.pose.pose,sizeof(tf::Pose));

        listener.transformPose("base_link",current,trans);

        out.x = trans.pose.position.x;
        out.y = trans.pose.position.y;
        out.theta = getYaw(trans.pose.orientation);

    }
    catch(tf::TransformException &exception){
        ROS_ERROR("%s", exception.what());
        ROS_INFO_STREAM("AMCL POSE TRANSFORMATION FAILED");
        return false;
    }

    return true;
}

bool Drive::convertToRobotFrame(const std::vector<Vector2D> &path, std::list<Vector2D> &vout){

    static tf::TransformListener listener;
    tf::StampedTransform transform_in_map;

    vout.clear();

    try{

        // bool got_transform = listener.waitForTransform("base_link","map",ros::Time(0),ros::Duration(3.0));
        listener.lookupTransform("map", "base_link", ros::Time(0), transform_in_map);

        /* no transform avaiable */
        //if(!got_transform) return false;

        /* transform global coordinates to base_link */
        for (auto p : path){

            tf::Vector3 coord(p.getX(),p.getY(),0);
            tf::Vector3 rotated = transform_in_map*coord;

            vout.push_back(Vector2D(rotated.getX(),rotated.getY()));

        }


    }
    catch(tf::TransformException &exception){
        ROS_ERROR("%s", exception.what());
        ROS_INFO_STREAM("PATH TRANSFORMATION FAILED");
        return false;
    }

    return true;
}

void Drive::drive(){

    /* nav_goal stores actual waypoint */
    Vector2D nav_goal;

    /* excecution time */
    ros::Rate rate(RATE);
    double dt = 1./RATE;

    /* Ramer-Douglas-Peucker*/
    RPD rdp;

    /* cruise control */
    CruiseControl cruise(nh_);
    //cruise.stop();

    /* RPD reduced path and transformed path */
    std::vector<Vector2D> approx;
    std::list<Vector2D> waypoints;

    /* keep track of robot pose */
    pose_t base_pose;

    /* start state of state machine */
    status_t state = IDLE;

    //PID angle(4.,0.,1.,dt,-TURTLEBOT_ANG_MAX_RPS,TURTLEBOT_ANG_MAX_RPS,TURTLEBOT_ANG_MAX_RPS,TURTLEBOT_ANG_MAX_RPS), vel(.9,0.,0.,0.,TURTLEBOT_VEL_MAX_MPS);
    PID angle(4.,0.,0.,-TURTLEBOT_ANG_MAX_RPS,TURTLEBOT_ANG_MAX_RPS);
    //PID vel(1.5,0.,0.,0.,TURTLEBOT_VEL_MAX_MPS);

    double x = 0, y = 0, theta = 0;
    double v = 0, w = 0;

    ROS_INFO_STREAM("DT: " << dt);

    while (ros::ok()){

        switch(state){

        /* wait for path and start signal*/
        case IDLE:

            if (!pose_.empty()){
                ROS_INFO_STREAM("NEW PATH RECEIVED");
                approx.clear();
                waypoints.clear();

                std::vector<Vector2D> path;

                for (const PoseStamped &p: pose_){
                    path.push_back(Vector2D(p.pose.position.x,p.pose.position.y));
                }

                mtx_.lock();
                rdp.approximate_path(path,approx,eps);
                for (auto wp : approx) waypoints.push_back(wp);
                /* remove first entry since this is the robot start position */
                waypoints.pop_front();
                mtx_.unlock();


#ifdef PUB_TO_RVIZ
                PublishMarker(approx);
#endif
                pose_.clear();

            }

            if(map_pose_ok){

                x = amcl_pose_.pose.pose.position.x;
                y = amcl_pose_.pose.pose.position.y;
                theta = getYaw(amcl_pose_.pose.pose.orientation);

            }

            /* check if new waypoints are avaiable and the robot should drive */
            if (!waypoints.empty() && drive_path && map_pose_ok){
                /* transform to base frame and update pose */
                //if(convertToRobotFrame(approx,transformed) && convertToRobotFrame(amcl_pose_,base_pose)){
                ROS_INFO_STREAM("FETCH WAYPOINT");
                map_pose_ok = false;

                state = FETCH_WAYPOINT;

                //TODO notify client if something failed
            }
            else state = IDLE;
            break;

        /* extract next checkpoint */
        case FETCH_WAYPOINT:

            if (waypoints.empty()){
                ROS_WARN_STREAM("ALL CHECKPOINTS REACHED, RETURN TO IDLE STATE");
                state = IDLE;
            }
            else
            {
                PublishCmdVel(0.,0.);
                nav_goal = waypoints.front();

                pose_t current, goal;

                current.x = x;
                current.y = y;
                current.theta = theta;

                goal.x = nav_goal.getX();
                goal.y = nav_goal.getY();

                if(cruise.moveTo(current,goal,0.1)){

                    state = DRIVING;
                    waypoints.pop_front();
                    cruise.start();
                    ROS_WARN_STREAM("NEW CHECKPOINT: " << nav_goal << " START DRIVING");

                }
                else state = FETCH_WAYPOINT;


                //state = DRIVING;
            }
            break;

        /* move turtlebot to checkpoint */
        case DRIVING:
#if 0
            x = amcl_pose_.pose.pose.position.x;
            y = amcl_pose_.pose.pose.position.y;
            theta = getYaw(amcl_pose_.pose.pose.orientation);
            //map_pose_ok = false;

            pose_t goal;

            goal.x = nav_goal.getX();
            goal.y = nav_goal.getY();

            //ROS_INFO_STREAM("ESTIMATED POSE: " << "[x y theta]: [" << x << " " << y << " " << theta << "]");
            //ROS_INFO_STREAM("CURRENT GOAL: " << "[x y]: [" << goal.x << " " << goal.y <<  "]");

            // control ctrl = pursuit.get_control(base_pose,goal);
            double diff_x = goal.x - x;
            double diff_y = goal.y - y;
            double dist = std::sqrt(diff_x*diff_x + diff_y*diff_y);
            ROS_INFO_STREAM("DIST TO GOAL: " << dist);

            if(dist < .2){
                ROS_WARN_STREAM("REACHED WAYPOINT");
                PublishCmdVel(0,0);
                v = w = 0;
                state = FETCH_WAYPOINT;
                break;
            }

            double angle_to_goal = std::atan2(diff_y,diff_x);
            double angle_diff = angle_to_goal - theta;

            v = std::fabs(angle_diff) > 75./180.*M_PI ? 0 : saturation(1.5*dist,0.,TURTLEBOT_VEL_MAX_MPS);
            w = angle.get_new_val(0.,angle_diff,dt);//saturation(4.*(angle_diff), -TURTLEBOT_ANG_MAX_RPS, TURTLEBOT_ANG_MAX_RPS);
            // ROS_INFO_STREAM("ANGLE DIFF: " << angle_diff);
            /* update position and orientation for next iteration */
            /*
            if (w != 0){
                double r = std::fabs(v/w);
                double w_dt = w*dt;
                x += r*( sin(theta + w_dt) - sin(theta) );
                y += r*( cos(theta) - cos(theta + w_dt) );
                theta += w_dt;
            }
            else{
                x += v*dt*cos(theta);
                y += v*dt*sin(theta);
            }
            */

            ROS_INFO_STREAM("ANGULAR: " << w << " SPEED: " << v);

            /* publish velocity commands */
            PublishCmdVel(w,v);

            //while(theta > 2*M_PI) theta -= 2*M_PI;
            //while(theta < 0) theta += 2*M_PI;
#endif
            if(cruise.reachedGoal()){
                ROS_INFO_STREAM("CRUISE CONTROL REACHED GOAL");
                state = FETCH_WAYPOINT;
                cruise.stop();
                break;
            }

            if(!drive_path){
                ROS_INFO_STREAM("ABORT REQUEST RECEIVED, RETURN TO IDLE STATE");
                state = IDLE;
                cruise.stop();
            }
            else state = DRIVING;

            break;
        }

        ros::spinOnce();
        rate.sleep();

    }

    run_thread_ = false;
    reset_IO();

}

