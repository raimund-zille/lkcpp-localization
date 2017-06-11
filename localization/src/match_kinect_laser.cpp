#include "ros/ros.h"
#include "localization/featureArrayH.h"
#include "geometry_msgs/Vector3.h"

///wie viel mehr kinect gegenüber laser gewichtet werden soll
#define WEIGHT_KINECT 1

localization::featureArrayH features;

static void callBackLaser(const localization::featureArrayHConstPtr &f){
    //copy laserscan
    ROS_INFO("GOT LASERFEATURES");
    for(size_t i = 0; i < f->features.size(); i++){
        localization::feature curr_f = f->features[i];
        curr_f.specifier = 1;
        features.features.push_back(curr_f);
    }
}

static void callBackKinect(const localization::featureArrayHConstPtr &f){
    //copy kinect
    ROS_INFO("GOT KINECTFEATURES");
    for(int j = 0; j < WEIGHT_KINECT; j++){
        for(size_t i = 0; i < f->features.size(); i++){
            localization::feature curr_f = f->features[i];
            curr_f.phi = curr_f.phi*M_PI/180;
            curr_f.r = curr_f.r;
            curr_f.specifier = 100;
            features.features.push_back(curr_f);
        }
    }
}



int main(int argc, char **argv){

    ros::init(argc,argv,"kinect_laser_match_node");
    ros::NodeHandle nh;

    ros::Rate rate(30);

    ros::Subscriber laser_sub = nh.subscribe("/laserFeatures", 10, &callBackLaser);
    ros::Subscriber kinect_sub = nh.subscribe("/featuresGreen", 10, &callBackKinect);
    ros::Publisher feature_pub = nh.advertise<localization::featureArrayH>("/features", 1);

    while(ros::ok()){
        features.header.stamp = ros::Time::now();
        feature_pub.publish(features);
        features.features.clear();
        ros::spinOnce();
        rate.sleep();
    }
}
