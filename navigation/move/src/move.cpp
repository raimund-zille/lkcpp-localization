#include <tf/tf.h>
#include <tf/exceptions.h>

/* C++ header files */
#include <algorithm>
#include <angles/angles.h>
#include <cmath>

/* costum header files */
#include "move.h"
#include "PID.h"

#define ALPHA_1 0.001
#define ALPHA_2 0.001
#define ALPHA_3 0.001
#define ALPHA_4 0.001

using namespace std;
using namespace angles;

/* friend function */
void PublishCmdVel(double angular, double speed, const ros::Publisher &vel_cmd){

    geometry_msgs::Twist msg;

    msg.angular.z = angular;
    msg.linear.x = speed;
    vel_cmd.publish(msg);

}

static double sample_normal_dist(double b){

    static std::random_device rd;
    static std::mt19937 generator(rd());

    //std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-b,b);
    double sum = 0;
    for (int i = 0; i < 12; i++){
        /* get probabilities in interval [-1, 1] */
        //sum += 2.*drand48() - 1.;

        sum += distribution(generator);
    }


    return sum / 2.;
}



void Move::setOdom(const nav_msgs::OdometryConstPtr &odom){

    static double theta_last = tf::getYaw(odom->pose.pose.orientation);

    theta_ = tf::getYaw(odom->pose.pose.orientation);
    dtheta_ = shortest_angular_distance(theta_,theta_last);
    theta_last = theta_;
    x_ = odom->pose.pose.position.x;
    y_ = odom->pose.pose.position.y;
    v_ = odom->twist.twist.linear.x;
    w_ = odom->twist.twist.angular.z;
    odom_recv_once_ = true;

}

/**** Turn Class ****/
Turn::Turn() : target_angle_(0), current_angle_(0), thresh_(0){

    x_ = y_ = 0;
    v_ = w_ = 0;
    dtheta_ = theta_ = 0;

    odom_recv_once_ = false;
    angleCtrl_.set_new_params(P_ANGLE,I_ANGLE,D_ANGLE,-TURTLEBOT_ANG_MAX_RPS,TURTLEBOT_ANG_MAX_RPS);

}

bool Turn::reachedGoal(){
    return fabs(fabs(target_angle_) - fabs(current_angle_)) < thresh_;
}

bool Turn::turnBy(double angle, double thresh){

    /* force angle to be in interval [0,2*PI] and check if it was negative */
    target_angle_ = fmod(fabs(angle),2*M_PI)*(angle < 0 ? -1. : 1.);
    current_angle_ = 0;
    thresh_ = thresh;
    //ROS_INFO_STREAM("theta: "  << normalize_angle_positive(theta_)<< " target angle: " << normalize_angle_positive(target_angle_));
    return odom_recv_once_;
}

void Turn::move(double dt, const ros::Publisher &vel_cmd){

    static double w = 0;

    if(!reachedGoal() && dt > 0. && odom_recv_once_)
    {
        /* update current angle depending in which direction the robot should have turned from previous iteration */
        current_angle_ +=  fabs(dtheta_) * (w < 0 ? -1 : 1);
        w = angleCtrl_.get_new_val(target_angle_,current_angle_,dt);

        ROS_INFO_STREAM("CURRENT ANGLE:  " << to_degrees(current_angle_) << " TARGET ANGLE: " << to_degrees(target_angle_) << " w: " << w << " DTHETA: " << to_degrees(dtheta_));

        PublishCmdVel(w,0,vel_cmd);

    }
    else
    {
        /* immediately stop robot and reset omega */
        PublishCmdVel(0,0,vel_cmd);
        w = 0;
    }


}


/**** MoveByDistAngle Class ****/
MoveByDistAngle::MoveByDistAngle(): pos_(0.,0.), goal_(0.,0.), diff_(0.,0.), thresh_(0){

    /* init variables of base class */
    x_ = y_ = 0;
    v_ = w_ = 0;
    dtheta_ = theta_ = 0;

    odom_recv_once_ = false;
    angleCtrl_.set_new_params(P_ANGLE,I_ANGLE,D_ANGLE,-TURTLEBOT_ANG_MAX_RPS,TURTLEBOT_ANG_MAX_RPS);
    velCtrl_.set_new_params(P_VEL,I_VEL,D_VEL,.1,TURTLEBOT_VEL_MAX_MPS);

}

bool MoveByDistAngle::reachedGoal(){
    return diff_.magnitute() < thresh_;
}

bool MoveByDistAngle::moveByDistAngle(double dist, double angle, double thresh){


    thresh_ = thresh;

    /* force angle to be in interval [0,2*PI] and check if it was negative */
    double target_angle = fmod(fabs(angle),2*M_PI)*(angle < 0 ? -1. : 1.);
    target_angle = normalize_angle(theta_ + target_angle);

    if(!odom_recv_once_) return false;

    pos_ = Vector2D(x_,y_);
    diff_ = dist*Vector2D(target_angle,RAD);
    ROS_WARN_STREAM("LENGTH TO GOAL: " << diff_.magnitute() << " WANTED LENGTH: " << dist);
    goal_ = pos_ + diff_;

    return true;
}

void MoveByDistAngle::move(double dt, const ros::Publisher &vel_cmd){

    if(!reachedGoal() && dt > 0 &&odom_recv_once_)
    {
        /* update position of robot */
        pos_ = Vector2D(x_,y_);

        /* get vector from robot to goal */
        diff_ = goal_ - pos_;

        /* force angle difference to 0 */
        double angle_to_goal =  atan2(diff_.getY(), diff_.getX());
        double shortest_angle = shortest_angular_distance(theta_,angle_to_goal);
        double w = angleCtrl_.get_new_val(shortest_angle,0., dt);

        /* if shortest angle is greater than 75 degrees, only rotate the robot */
        double v = fabs(shortest_angle) > (50./180.*M_PI) ? 0 : velCtrl_.get_new_val(diff_.magnitute(),v_,dt);

        //ROS_INFO_STREAM("ANGLE GOAL, CURRENT: [" << to_degrees(angle_to_goal) << " " << to_degrees(theta_) << "]");
        ROS_INFO_STREAM("GOAL: " << goal_ << " CURRENT: " << pos_);
        PublishCmdVel(w,v,vel_cmd);
    }
    else
    {
        /* immediately stop the robot */
        PublishCmdVel(0,0,vel_cmd);

    }
}

MoveToPos::MoveToPos() : pos_(), goal_(), diff_(0,0), thresh_(0){

    /* init variables of base class */
    x_ = y_ = 0;
    v_ = w_ = 0;
    dtheta_ = theta_ = 0;

    odom_recv_once_ = false;
    angleCtrl_.set_new_params(P_ANGLE,I_ANGLE,D_ANGLE,-TURTLEBOT_ANG_MAX_RPS,TURTLEBOT_ANG_MAX_RPS);
    velCtrl_.set_new_params(P_VEL,I_VEL,D_VEL,.1,TURTLEBOT_VEL_MAX_MPS);

}

bool MoveToPos::moveFromPosToGoal(const pose_t &start, const pose_t &goal, double thresh){

    pos_ = start;
    goal_ = goal;
    thresh_ = thresh;
    diff_ = Vector2D(goal_.x - pos_.x, goal_.y - pos_.y);

    return odom_recv_once_;

}

bool MoveToPos::reachedGoal(){
    return diff_.magnitute() < thresh_;
}

void MoveToPos::move(double dt, const ros::Publisher &vel_cmd){

    static double x_prev = x_, y_prev = y_, theta_prev = theta_;

    if(!reachedGoal() && dt > 0 &&odom_recv_once_)
    {
        /* update robots position and pray that it is accurate =D */
        /*
        double dtheta = fabs(dtheta_) * (w_ < 0 ? -1 : 1);

        if (w_ != 0){
            double r = std::fabs(v_/w_);
            // double w_dt = w*dt;
            pos_.x += r*( sin(pos_.theta + dtheta) - sin(pos_.theta) );
            pos_.y += r*( cos(pos_.theta) - cos(pos_.theta + dtheta) );
            pos_.theta += dtheta;
            pos_.theta = normalize_angle(pos_.theta);
        }
        else{
            pos_.x += v_*dt*cos(pos_.theta);
            pos_.y += v_*dt*sin(pos_.theta);
        }

        double xdiff = goal_.x - pos_.x;
        double ydiff = goal_.y - pos_.y;
        double angle_to_goal = atan2(ydiff,xdiff);

        double shortest_angle = shortest_angular_distance(theta_,angle_to_goal);
        double w = angleCtrl_.get_new_val(shortest_angle,0., dt);
        double v = fabs(shortest_angle) > (75/180.*M_PI) ? 0 : velCtrl_.get_new_val(diff_.magnitute(),v_,dt);

        ROS_WARN_STREAM("CURRENT POSE: " << pos_ << "GOAL: " << goal_);


        PublishCmdVel(w,v,vel_cmd);

        diff_ = Vector2D(xdiff,ydiff);
        ROS_WARN_STREAM("DIST TO GOAL: " << diff_.magnitute());
        */
        double dy = y_ - y_prev;
        double dx = x_ - x_prev;
        double d_rot_1 = atan2(dy,dx)- theta_prev;
        double d_rot_2 = theta_- theta_prev - d_rot_1;
        double d_trans = sqrt(dy*dy+dx*dx);


        double d_rot_prime_1 = d_rot_1 - sample_normal_dist(ALPHA_1*d_rot_1+ALPHA_2*d_trans);
        double d_rot_prime_2 = d_rot_2 - sample_normal_dist(ALPHA_1*d_rot_2+ALPHA_2*d_trans);
        double d_trans_prime = d_trans - sample_normal_dist(ALPHA_3*d_trans+ALPHA_4*(d_rot_1+d_rot_2));


        pos_.x += d_trans_prime*cos(pos_.theta + d_rot_prime_1);//d_rot_prime_1);
        pos_.y += d_trans_prime*sin(pos_.theta + d_rot_prime_1);
        pos_.theta += d_rot_prime_1+d_rot_prime_2;
        pos_.theta = normalize_angle(pos_.theta);

        diff_ = Vector2D(goal_.x - pos_.x,goal_.y - pos_.y);

        double angle_to_goal = atan2(diff_.getY(),diff_.getX());

        double shortest_angle = shortest_angular_distance(pos_.theta,angle_to_goal);
        double w = angleCtrl_.get_new_val(shortest_angle,0., dt);
        double v = fabs(shortest_angle) > (50./180.*M_PI) ? 0 : velCtrl_.get_new_val(diff_.magnitute(),v_,dt);

        ROS_WARN_STREAM("CURRENT POS: " << pos_ << "GOAL: " << goal_);


        PublishCmdVel(w,v,vel_cmd);

        y_prev = y_;
        x_prev = x_;
        theta_prev = theta_;
    }
    else
    {
        /* immediately stop the robot */
        PublishCmdVel(0,0,vel_cmd);

    }

}


MoveWithOdom::MoveWithOdom(): goal_(0.,.0), thresh_(0.), diff_(0.,0.){

    /* init variables of base class */
    x_ = y_ = 0;
    v_ = w_ = 0;
    dtheta_ = theta_ = 0;

    odom_recv_once_ = false;
    angleCtrl_.set_new_params(P_ANGLE,I_ANGLE,D_ANGLE,-TURTLEBOT_ANG_MAX_RPS,TURTLEBOT_ANG_MAX_RPS);
    velCtrl_.set_new_params(P_VEL,I_VEL,D_VEL,.1,TURTLEBOT_VEL_MAX_MPS);

}

bool MoveWithOdom::reachedGoal(){
    return diff_.magnitute() < thresh_;
}

bool MoveWithOdom::moveToOdomPos(const Vector2D &goal,double thresh){

    goal_ = goal;
    thresh_ = thresh;
    diff_ = Vector2D(goal.getX() - x_, goal.getY() - y_);
    //ROS_INFO_STREAM("Diff " << angles::to_degrees(atan2(diff_.getY(), diff_.getX())) << " distance " << diff_.magnitute());
    return odom_recv_once_;
}

void MoveWithOdom::move(double dt, const ros::Publisher &vel_cmd){

    static Vector2D pos(x_,y_);

    if(!reachedGoal() && dt > 0 && odom_recv_once_)
    {
        /* update position of robot */
        pos = Vector2D(x_,y_);

        /* get vector from robot to goal */
        diff_ = goal_ - pos;

        /* force angle difference to 0 */
        double angle_to_goal =  atan2(diff_.getY(), diff_.getX());
        double shortest_angle = shortest_angular_distance(theta_,angle_to_goal);
        double w = angleCtrl_.get_new_val(shortest_angle,0., dt);

        /* if shortest angle is greater than 75 degrees, only rotate the robot */
        double v = fabs(shortest_angle) > (50./180.*M_PI) ? 0 : velCtrl_.get_new_val(diff_.magnitute(),v_,dt);

        //ROS_INFO_STREAM("ANGLE GOAL, CURRENT: [" << to_degrees(angle_to_goal) << " " << to_degrees(theta_) << "]");
        //ROS_INFO_STREAM("GOAL: " << goal_ << " CURRENT: " << pos);
        PublishCmdVel(w,v,vel_cmd);
    }
    else
    {
        /* immediately stop the robot */
        PublishCmdVel(0,0,vel_cmd);
        pos = Vector2D(x_,y_);

    }

}





