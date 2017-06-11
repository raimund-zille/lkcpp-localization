/* ROS header files */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

/* costum header files */
#include "../drive/inc/drive.h"


int main(int argc, char **argv){

    ros::init(argc,argv,"drive_node");

    ros::NodeHandle n;
    Drive driver(n);

    driver.drive();

}
