/* ROS header files*/
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

/* AMCL header files */
//#include "../amcl/inc/motion_model.h"
//#include "../amcl/inc/amcl.h"
//#include "../amcl/inc/measurement_model.h"
//#include "../amcl/inc/landmarks.h"

/* Message header files */
#include "localization/featureArray.h"
#include "localization/featureArrayH.h"

//#include <random>



#define R_POLE 0.06
#define THRESHOLD_SAME_OBJ_R 0.1   //abweichung von 10 cm vor/zurück
#define THRESHOLD_SAME_OBJ_A 1.5*0.0174532923847+0.003
  //abweichung von ca 3 grad links/rechts
#define THRESHOLD_MAX_R 3         //only trust if not further away then 5 m
ros::Publisher feature_pub;
ros::Subscriber laser_sub;
localization::featureArrayH feature_array;
bool debug = false;

void laser_sb(const sensor_msgs::LaserScanConstPtr &scan){
    if(debug)ROS_INFO("STARTING LASERFEATUREDETECTION");
    /* clean up */
    feature_array.features.clear();
    feature_array.header.frame_id = "/base_link";
    feature_array.header.stamp = ros::Time::now();

    /* init */

    double angle_min = scan->angle_min;
    double angle_increment = scan->angle_increment;

    double prev_range = scan->ranges[0];
    double curr_range = 0;
    double angle_range = 0; //curr angle of min/maxima of same ranges

    for(size_t i = 1; i < scan->ranges.size(); i++){
        curr_range = scan->ranges[i];
        if(fabs(curr_range-prev_range)< THRESHOLD_SAME_OBJ_R){

            if(isinf(curr_range)){
                if(debug)ROS_INFO("NAN");
                //NAN or INF -> recet
                angle_range = 0;
                prev_range = curr_range;
                continue;
            }
            if(debug)ROS_INFO("SAME OBJECT!");
            //same object hurray
            angle_range += angle_increment;
        }else{
            if(debug)ROS_INFO("NEW PEAK!");
            //new peak
            //1. check prev peak
            double desired_angle_range = atan2(R_POLE, prev_range);
            if(fabs(desired_angle_range - angle_range)< THRESHOLD_SAME_OBJ_A){
                //feature detected!
                double angle_f = angle_min + (i-1)*angle_increment + M_PI; //offset 0 deg looks behind him
                double range_f = prev_range;
                if(isinf(range_f) || range_f > THRESHOLD_MAX_R){
                    //recect!
                    if(debug)ROS_INFO("NAN");
                    angle_range = 0;
                    prev_range = curr_range;
                    continue;
                }
                ROS_INFO_STREAM("FEATURE! r: " << prev_range << " phi: " << angle_f);
                localization::feature curr_f;
                curr_f.knownCorr = -1;
                curr_f.specifier = 200; //GREEN
                curr_f.phi = angle_f;
                curr_f.r = range_f;
                feature_array.features.push_back(curr_f);
            }

            //2. clean up
            angle_range = 0;
        }
        //set prev_range
        prev_range = curr_range;
        if(debug)ROS_INFO_STREAM("Range: " << curr_range);
    }

    feature_pub.publish(feature_array);
    ROS_INFO_STREAM("Time: " << ros::Time::now() - feature_array.header.stamp);

}


int main(int argc, char **argv){

    ros::init(argc,argv,"feature_node");
    ros::NodeHandle nh;

    ros::Rate rate(30);

    laser_sub = nh.subscribe("/laserscan", 1, &laser_sb);
    feature_pub = nh.advertise<localization::featureArrayH>("/laserFeatures", 1);

    while(ros::ok()){

        ros::spinOnce();
        rate.sleep();
    }
}
