/* ROS header files */
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <navigation/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>

/* C++ header files */
#include <string>

using namespace geometry_msgs;

Pose2D start, goal;

bool start_ok = false, goal_ok = false;

static void get_rviz_pose(const geometry_msgs::PoseStampedConstPtr &goal_pose){

    goal.theta = tf::getYaw(goal_pose->pose.orientation);
    goal.x = goal_pose->pose.position.x;
    goal.y = goal_pose->pose.position.y;

    goal_ok = true;
}

static void get_rviz_init_pose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start_pose){

    start.theta = tf::getYaw(start_pose->pose.pose.orientation);
    start.x = start_pose->pose.pose.position.x;
    start.y = start_pose->pose.pose.position.y;

    start_ok = true;
}

static void get_amcl_pose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose){

    start.theta = tf::getYaw(amcl_pose->pose.pose.orientation);
    start.x = amcl_pose->pose.pose.position.x;
    start.y = amcl_pose->pose.pose.position.y;

    start_ok = true;
}

/* wait for setup data */
static void wait_for_data(){
    while((!start_ok ||!goal_ok) && ros::ok())
        ros::spinOnce();
}

int main(int argc, char **argv){

    ros::init(argc,argv,"test_pathplanner_node");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<navigation::Path::Request, navigation::Path::Response>("GetPath");
    navigation::Path srv;
    ros::Subscriber goal_pose_sub = n.subscribe("move_base_simple/goal",1 ,get_rviz_pose);
    // ros::Subscriber init_pose_sub = n.subscribe("/initialpose",1,get_rviz_init_pose);
    ros::Subscriber init_pose_sub = n.subscribe("/amcl_pose",1,get_rviz_init_pose);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("navigation/mypath",1);

    ros::Rate r(5);
    ROS_INFO_STREAM("WAIT FOR DATA");
    wait_for_data();
    ROS_INFO_STREAM("TEST PATHPLANNING STARTET");

    /* stop callbacks */
    goal_pose_sub.shutdown();
    init_pose_sub.shutdown();

    bool published_path = false;

    while (ros::ok() && !published_path){



        /*
         memcpy(&srv.request.goal,&goal, sizeof(Pose2D));
         memcpy(&srv.request.start,&start,sizeof(Pose2D));
         */

        srv.request.goal = goal;
        srv.request.start = start;

        if(client.call(srv)){

            nav_msgs::Path path;

            for (auto p : srv.response.poses){
                path.poses.push_back(p);
            }


            path.header.stamp = ros::Time::now();
            path.header.frame_id = "/map";

            path_pub.publish(path);
            ROS_INFO_STREAM("PUBLISHED PATH, SHUTTING DOWN...");
            published_path = true;

        }
        else ROS_INFO_STREAM("FAILED TO GET PATH");

        // goal_ok = start_ok = false;


        // ros::spinOnce();
        r.sleep();
    }
}
