#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ros::Time laser_last_call;

static void laser_callback(const sensor_msgs::LaserScanConstPtr &data){
    laser_last_call = ros::Time::now();
}

void restart_laser(){
    system("rosnode kill /rplidarNode");
    ros::Rate rate(1);
    rate.sleep();
    system("roslaunch rplidar_ros rplidar.launch");
    ROS_INFO("restarted rplidar!");
    laser_last_call = ros::Time::now();
}


int main(int argc, char **argv){
    ros::init(argc, argv, "restart_node");
    ros::NodeHandle nh;
    ros::Rate rate(1);

    ros::Subscriber laser_sub = nh.subscribe("laserscan",1,laser_callback);
    ros::Duration restart_laser_dur(10.0);
    while(ros::ok()){
        ros::spinOnce();
        if(ros::Time::now()-laser_last_call > restart_laser_dur){
            restart_laser();
        }
        rate.sleep();
    }

    return 0;
}
