/* ROS header files*/
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

/* CLUSTER file */
#include "../include/cluster/cluster.h"

/* AMCL header files */
#include "motion_model.h"
#include "amcl.h"
#include "measurement_model.h"
#include "landmarks.h"

/* Message header files */
#include "localization/featureArray.h"
#include "localization/featureArrayH.h"

/* sync messages */
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include <random>

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, localization::featureArrayH> SyncPolicy;

int8_t *map;
bool map_received = false;
bool odom_received = false;
int width = 1000;
int height = 1000;
float resolution = 0;
std::vector<feature> features;
std::vector<feature> showFeatures;
double estimatedPos_received;
geometry_msgs::PoseArray featureDetected;
geometry_msgs::PoseArray featureDetected2;
pose_t desiredPos;
pose_t estimatedPos;
pose_t pose;
bool use_scan_detection = false;
bool debug = false;
bool test = false;
bool use_sync = false;
bool publish_rviz = true;
bool publish_time = false;

bool a_b_recieved = false;
double a = 0;
double b = 0;
int color = 0;

/**
 * @brief featureInWorld transformation for feature in base_link to map koordinates
 * @param f feature in base_link
 * @return feature in map
 */
geometry_msgs::Pose featureInWorld(feature f){
    geometry_msgs::Pose returnPose;
    double phi = f.phi + desiredPos.theta;
    double x = desiredPos.x + cos(phi)*f.r;
    double y = desiredPos.y + sin(phi)*f.r;
    returnPose.position.x = x;
    returnPose.position.y = y;
    return returnPose;
}

/**
 * @brief get_map call_back for map
 * @param m map
 */
static void get_map(const nav_msgs::OccupancyGridConstPtr &m){


    width = m->info.width;
    height = m->info.height;
    resolution = m->info.resolution;

    size_t size = width*height;

    map = (int8_t*) calloc(size,sizeof(int8_t));

    for (size_t i = 0; i < size; i++){
        map[i] = m->data[i];
    }

    map_received = true;


}

/**
 * @brief featuresb call_back for features (not sync)
 * @param featuresMsg incoming features
 */
static void featuresb(const localization::featureArrayHConstPtr &featuresMsg){
    if(publish_time)ROS_INFO_STREAM("Features receieved! TimeStamp: " << featuresMsg->header.stamp);
    //features.clear();
    for(size_t i = 0; i < featuresMsg->features.size(); i++){
        localization::feature locDummyFeature = featuresMsg->features[i];
        feature dummyFeature;
        dummyFeature.knownCorr  = locDummyFeature.knownCorr;
        dummyFeature.phi        = locDummyFeature.phi;
        dummyFeature.r          = locDummyFeature.r;
        dummyFeature.specifier  = locDummyFeature.specifier;
        features.push_back(dummyFeature);
    }
    //features.clear(); // UNDO
}

/**
 * @brief estimatedPossb call_back for estimatedPos, if published set robot pose on estimatedPos
 * @param estimatedPosP estimated position where robot is
 */
static void estimatedPossb(const geometry_msgs::PoseConstPtr &estimatedPosP){
    estimatedPos.x = estimatedPosP->position.x;
    estimatedPos.y = estimatedPosP->position.y;
    estimatedPos.theta = tf::getYaw(estimatedPosP->orientation);
    ROS_INFO_STREAM("estimatedPos received! " << estimatedPos.x << "/" << estimatedPos.y << "/" << estimatedPos.theta);
    estimatedPos_received = true;
}

static void estimatedPosRvizsb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &estimatedPosP){
    estimatedPos.x = estimatedPosP->pose.pose.position.x;
    estimatedPos.y = estimatedPosP->pose.pose.position.y;
    estimatedPos.theta = tf::getYaw(estimatedPosP->pose.pose.orientation);
    ROS_INFO_STREAM("estimatedPos received! " << estimatedPos.x << "/" << estimatedPos.y << "/" << estimatedPos.theta);
    estimatedPos_received = true;
}

/**
 * @brief odom_cb call_back for odometry
 * @param odom incoming odometry
 */
static void odom_cb(const nav_msgs::Odometry::ConstPtr &odom){
    if(publish_time)ROS_INFO_STREAM("Odom receieved! TimeStamp: " << odom->header.stamp);
    pose.x = odom->pose.pose.position.x;
    pose.y = odom->pose.pose.position.y;
    pose.theta = tf::getYaw(odom->pose.pose.orientation);
    odom_received = true;
}

/**
 * @brief test_odom_cb call_back for test odom published by test_node
 * @param odom incoming test odom
 */
static void test_odom_cb(const geometry_msgs::PoseStampedConstPtr &odom){
    pose.x = odom->pose.position.x;
    pose.y = odom->pose.position.y;
    pose.theta = tf::getYaw(odom->pose.orientation);
    odom_received = true;
}

/**
 * @brief wait_for_map waits till map received
 * @param n nodehandle
 */
static void wait_for_map(ros::NodeHandle &n){

    ros::Subscriber map_sub =  n.subscribe("/map",1, get_map);
    // ros::Rate r(10);

    ROS_INFO_STREAM("waiting for map...");

    while(ros::ok() && !map_received){
        ros::spinOnce();
        // r.sleep();

    }

    ROS_INFO_STREAM("map received!");
}

static void a_b_cb(const geometry_msgs::Vector3ConstPtr &ab){
    ROS_INFO("got a b");
    a = ab->x;
    b = ab->y;
    color = (int)ab->z; //0 blue, 1 yellow
    a_b_recieved = true;
}

/**
 * @brief syncCallback call_back for odometry and features (sync)
 * @param odom incoming odometry
 * @param featuresMsg incoming features
 */
static void syncCallback(const nav_msgs::Odometry::ConstPtr &odom, const localization::featureArrayHConstPtr &featuresMsg){
    ROS_INFO("Sync callback");
    //ODOM
    pose.x = odom->pose.pose.position.x;
    pose.y = odom->pose.pose.position.y;
    pose.theta = tf::getYaw(odom->pose.pose.orientation);
    odom_received = true;

    //FEATURES
    features.clear();
    for(size_t i = 0; i < featuresMsg->features.size(); i++){
        localization::feature locDummyFeature = featuresMsg->features[i];
        feature dummyFeature;
        dummyFeature.knownCorr  = locDummyFeature.knownCorr;
        dummyFeature.phi        = locDummyFeature.phi;
        dummyFeature.r          = locDummyFeature.r;
        features.push_back(dummyFeature);
    }
}



int main(int argc, char **argv){

    /* Iinitialise everything */

    /* ROS NODE */
    ros::init(argc,argv,"localization_node");
    ros::NodeHandle nh;
    ros::Rate rate(10);


    /* MAP */
    //subscriber for a, b
    //wait till received
    a_b_recieved = false;
    ros::Subscriber a_b_sub;
    a_b_sub = nh.subscribe("fieldDim", 100, a_b_cb);

    /* SUBSCRIBER FOR ODOMETRY */
    ros::Subscriber odom_sub; //async
    message_filters::Subscriber<nav_msgs::Odometry>* odom_sub_f; //sync

    if(test){
        ROS_INFO("ODOM SUBSCRIBER TEST");
        odom_sub = nh.subscribe("testOdom", 10, test_odom_cb);
    }else{
        if(!use_sync){
            ROS_INFO("ODOM SUBSCRIBER NO SYNC");
            odom_sub = nh.subscribe("odom",10,odom_cb);
        }else{
            ROS_INFO("ODOM SUBSCRIBER SYNC");
            odom_sub_f = new message_filters::Subscriber<nav_msgs::Odometry>(nh, "odom", 3);
        }
    }


    /* SUBSCRIBER FOR FEATURES */
    ros::Subscriber feature_sub; //async
    message_filters::Subscriber<localization::featureArrayH>* feature_sub_f; //sync
    feature_sub = nh.subscribe("features", 10, featuresb);

#if 0
    if(use_scan_detection){
        if(!use_sync){
            ROS_INFO("FEATURE SUBSCRIBER SCAN NO SYNC");
            feature_sub = nh.subscribe("featuresScan", 1, featuresb);
        }else{
            ROS_INFO("FEATURE SUBSCRIBER SCAN SYNC");
            feature_sub_f = new message_filters::Subscriber<localization::featureArrayH>(nh, "featuresScan", 3);
        }
    }else{
        ROS_INFO("FEATURE SUBSCRIBER KINECT");
        feature_sub = nh.subscribe("features", 10, featuresb);
    }
#endif
#if 0
    /*sync*/
    message_filters::Synchronizer<SyncPolicy>* sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(20), *odom_sub_f, *feature_sub_f);
    if(use_sync){
        sync->registerCallback(boost::bind(&syncCallback, _1, _2));
    }
#endif


    /* SUBSCRIBER FOR ESTIMATED POS */
    ros::Subscriber estimatedPos_sub = nh.subscribe("estimatedPos", 10 ,estimatedPossb);
    ros::Subscriber estimatedPosRviz_sub = nh.subscribe("initialpose", 10, estimatedPosRvizsb);


    /* PUBLISHER */
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseArray>("myparticlecloud",10);
    ros::Publisher landmark_pub = nh.advertise<geometry_msgs::PoseArray>("myLandmarks", 10);
    ros::Publisher cluster_pub = nh.advertise<geometry_msgs::PoseArray>("myPositionCloud",10);
    ros::Publisher feature_pub  = nh.advertise<geometry_msgs::PoseArray>("myFeatures",10);
    ros::Publisher feature_pub2  = nh.advertise<geometry_msgs::PoseArray>("myFeatures2",10);

#if 1
    ros::Rate wait_ab(1);
    while(!a_b_recieved){
        //wait for a,b
        if(!ros::ok()){
            return 0;
        }
        ros::spinOnce();
        wait_ab.sleep();
    }
#else
    //TODO: find length a and b of field
    a = 1.2;  //TESTFELD VALUES
    b = 2.95; //TESTFELD VALUES
#endif
    Landmarks landmarks(a, b);


    /* Init Agumented Monte Carlo Localization */
    AMCL amcl(3*a, b, 1000);


    /* Create motion- and measurementmodel */
    MotionModelOdom motion_model;
    MeasurementModel measure_model(landmarks);



    /* Declare variables to safe Particles */
    ParticleList estimated_pose;
    free(map);
    Cluster cluster;


    ROS_INFO_STREAM("Starting AMCL!");
    ros::Time time;


    /* WHILE LOOP
     * 1. check if a estimated position was received, if distribute around that position
     * 2. AMCL
     * 3. publish Particles (optional, for visual purpose)
     * 4. cluster Particles and publish most likely Position
     * 5. publish Landmarks and Features in map koordiantes (optional, for visual purpose)
     */
    while(ros::ok()){

        time = ros::Time::now();

        ros::spinOnce();


        /* 1. check if a estimated position was received */
        if(estimatedPos_received){
            //distribute around that position
            amcl.distributeAroundPosition(estimatedPos);
            estimatedPos_received = false;
        }


        if (!odom_received){
            ROS_WARN("NO ODOM"); //dieser teil ist meiner meinung nach unnötig @Gerhardt
            //continue;
        }
        odom_received = false;

        /* 2. AMCL */
        ros::Time timeamcl = ros::Time::now();
        amcl.augmentedMCL(pose, motion_model, measure_model, features, estimated_pose);
        if(publish_rviz) showFeatures = features;
        features.clear();
        if(publish_time)ROS_INFO_STREAM("AMCL Time: " << ros::Time::now() - timeamcl);


        /* 3. publish Particles for visual purpose */
        if(publish_rviz){
            geometry_msgs::PoseArray pose_array, landmark_array;

            //Particles
            for (auto j = 0; j < estimated_pose.getSize(); j++){
                pose_t i = estimated_pose.getParticleAt(j).pose;
                tf::Quaternion orientation = tf::createQuaternionFromYaw(i.theta);
                geometry_msgs::Pose current;
                current.position.x = i.x;
                current.position.y = i.y;
                memcpy(&current.orientation,&orientation,sizeof(tf::Quaternion));

                pose_array.poses.push_back(current);
            }

            //Landmarks
            for(landmark lm : landmarks.getList()){
                geometry_msgs::Pose curr;
                curr.position.x = lm.x;
                curr.position.y = lm.y;
                landmark_array.poses.push_back(curr);
            }
            pose_array.header.frame_id = "/map";
            pose_array.header.stamp = ros::Time::now();

            landmark_array.header.frame_id = "/map";
            landmark_array.header.stamp = ros::Time::now();


            pose_pub.publish(pose_array);
            landmark_pub.publish(landmark_array);
        }


        /* 4. cluster Particles to find most likley position */
        geometry_msgs::PoseArray cluster_array;
        ros::Time timec = ros::Time::now();

        //cluster
        ParticleList clusterList = cluster.get_mostlikli_pos(estimated_pose);
        clusterList.sort();

        //publish
        for(auto i = 0; i < clusterList.getSize(); i++){
            ROS_INFO_STREAM("i: " << i << " , weight: " << clusterList.getParticleAt(i));
            Particle currC = clusterList.getParticleAt(i);
            geometry_msgs::Pose current;
            current.position.x = currC.pose.x;
            current.position.y = currC.pose.y;
            tf::Quaternion orientation = tf::createQuaternionFromYaw(currC.pose.theta);
            memcpy(&current.orientation,&orientation,sizeof(tf::Quaternion));
            cluster_array.poses.push_back(current);

        }
        if(publish_time)ROS_INFO_STREAM("Cluster Time: " << ros::Time::now()-timec);
        cluster_array.header.frame_id = "/map";
        cluster_array.header.stamp = ros::Time::now();
        cluster_pub.publish(cluster_array);
        if(debug)measure_model.get_weight(clusterList.getParticleAt(0).pose, features, true);
        if(clusterList.getSize() == 1){
            amcl.set_num_particles(1000);
        }

        /* 5. Publish detected features */
        if(publish_rviz){
            desiredPos = clusterList.getParticleAt(0).pose;
            featureDetected.poses.clear();
            featureDetected2.poses.clear();
            featureDetected.header.frame_id = "/map";
            featureDetected2.header.frame_id = "/map";
            featureDetected.header.stamp = ros::Time::now();
            for(auto i = 0; i < showFeatures.size(); i++){
                if(showFeatures[i].specifier == 100)
                {
                    featureDetected.poses.push_back(featureInWorld(showFeatures[i]));
                }else{
                    featureDetected2.poses.push_back(featureInWorld(showFeatures[i]));
                }
            }
            feature_pub.publish(featureDetected);
            feature_pub2.publish(featureDetected2);
        }

        if(publish_time)ROS_INFO_STREAM("Time for 1 loop: " << ros::Time::now() - time);
        features.clear();
        rate.sleep();

    }

    return 0;
}
