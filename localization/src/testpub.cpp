/* ROS header files*/
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

/* AMCL header files */
#include "../amcl/inc/motion_model.h"
#include "../amcl/inc/amcl.h"
#include "../amcl/inc/measurement_model.h"
#include "../amcl/inc/landmarks.h"

/* Message header files */
#include "localization/featureArray.h"

#include <random>
#include <iostream>

ros::Publisher feature_pub;
ros::Publisher odom_pub;
ros::Publisher estimatedPos_pub;

double x, y, phi;
double a = 1.4;  //TESTFELD VALUES
double b = 2.85; //TESTFELD VALUES

int timestep;

static double add_gaussian_noise(double mean, double stdev){

    static std::random_device rd;
    static std::mt19937 generator(rd());

    std::normal_distribution<double> gauss(mean,stdev);

    return gauss(generator);

}

void motionCircle(double r){
    if(timestep == 360){
        timestep = 0;
    }
    double theta = ((double)timestep)*M_PI/180;
    x = 1.5*a + cos(theta)*r;
    y = 0.5*b + sin(theta)*r;
    phi = theta+M_PI_2;
    if(phi > M_PI){
        phi = phi - 2*M_PI;
    }
    timestep++;
}

feature rPhi(landmark lm){
    double r_des = 0;
    double phi_des = 0;

    double x_diff = lm.x - x;
    double y_diff = lm.y - y;

    r_des = sqrt(pow(x_diff,2)+pow(y_diff,2));
    phi_des = atan2(y_diff, x_diff);

    feature f;
    f.r = r_des;
    f.phi = phi_des - phi;
    f.knownCorr = -1;
    f.specifier = GREEN;

    return f;
}


int main(int argc, char **argv){

    ros::init(argc,argv,"test_node");
    ros::NodeHandle nh;

    estimatedPos_pub = nh.advertise<geometry_msgs::Pose>("estimatedPos",10);
    /* PUBLISH ONLY 1 ESTIMATED POS */

    double x;
    double y;
    double phi;
    std::cout << "What is my estimated Pos?" << std::endl;
    std::cout << "x: ";
    std::cin >> x;
    std::cout << "y: ";
    std::cin >> y;
    std::cout << "phi: ";
    std::cin >> phi;
    std::cout << "Got estimatedPos: (" << x << "/" << y << "/" << phi << ")" << std::endl;

    //build message

    geometry_msgs::Pose estimatedPos;
    estimatedPos.position.x = x;
    estimatedPos.position.y = y;
    tf::Quaternion q = tf::createQuaternionFromYaw(phi*M_PI/180);
    memcpy(&estimatedPos.orientation,&q,sizeof(tf::Quaternion));
    ROS_INFO("publish estimatedPos");
    ros::Rate rate2(10);
    for(int i = 0; i < 10; i++){
        estimatedPos_pub.publish(estimatedPos);
        if(!ros::ok()){
            return 0;
        }
        rate2.sleep();
    }
    ROS_INFO("Exiting...");
    return 0;



























    ros::Rate rate(5);

    //TODO: find length a and b of field

    Landmarks landmarks(a, b);
    x = 1.5*a;
    y = 0.5*b;
    phi = M_PI_2;
    timestep = 0;

    int counter = 0;
    bool estimated_pose_only = true;

    feature_pub = nh.advertise<localization::featureArray>("features",1);
    odom_pub = nh.advertise<geometry_msgs::PoseStamped>("testOdom", 1);
    estimatedPos_pub = nh.advertise<geometry_msgs::Pose>("estimatedPos",1);


    ROS_INFO_STREAM("Starting publishing testdata");

    while(ros::ok()){

        if(estimated_pose_only){
            geometry_msgs::Pose estimatedPos;
            estimatedPos.position.x = 0.25*a;
            estimatedPos.position.y = 0.5*b;
            estimatedPos.orientation.w = 1;
            ROS_INFO("publish estimatedPos");
            estimatedPos_pub.publish(estimatedPos);
            ros::spinOnce();
            rate.sleep();
            continue;
        }
        if(counter == 100){
            ROS_INFO("publish estimatedPos");
            //estimate your pos
            geometry_msgs::Pose estimatedPos;
            estimatedPos.position.x = x;
            estimatedPos.position.y = y;
            tf::Quaternion orientation = tf::createQuaternionFromYaw(phi);
            memcpy(&estimatedPos.orientation,&orientation,sizeof(tf::Quaternion));
            estimatedPos_pub.publish(estimatedPos);
            counter++;
        }else{
            counter++;
            if(counter == 10000){
                counter = 0;
            }
        }
#if 1
        //motion
        geometry_msgs::PoseStamped pubP;
        motionCircle(0.5);
        pubP.pose.position.x = x; + add_gaussian_noise(0, 0.01);
        pubP.pose.position.y = y + add_gaussian_noise(0, 0.1);

        pubP.header.stamp = ros::Time::now();
        pubP.header.frame_id ="/map";

        tf::Quaternion orientation = tf::createQuaternionFromYaw(phi);
        memcpy(&pubP.pose.orientation,&orientation,sizeof(tf::Quaternion));

        ROS_INFO_STREAM("PUBLISHING POSE! Current at(x/y/phi): (" << x << "/" << y << "/" << phi <<")");
        odom_pub.publish(pubP);
#endif



        //observation
        double range_sensor = M_PI_4;
        localization::featureArray pubF;
        //ROS_INFO_STREAM("x: " << x << " y: " << y <<" phi: " << phi);
        for(landmark lm : landmarks.getList()){
            if(lm.specifier == GREEN){
                localization::feature f;
                feature curr = rPhi(lm);

                f.phi = curr.phi;// + add_gaussian_noise(0.,0.1*M_PI);// - phi;
                f.r = curr.r;// + add_gaussian_noise(0,0.01);

                f.knownCorr = -1;//curr.knownCorr;
                f.specifier = curr.specifier;

                if(f.phi > - range_sensor && f.phi < range_sensor){
                    ROS_INFO_STREAM("Publishing observation: phi: " << f.phi << " r: " << f.r);
                    pubF.features.push_back(f);
                }
            }
        }



        ROS_INFO_STREAM("PUBLISHING FEATURES!");
        feature_pub.publish(pubF);



        ros::spinOnce();

        rate.sleep();

    }

    return 0;
}
