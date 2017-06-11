#include "motion_model.h"
#include <stdlib.h>
#include <ros/ros.h>

#include <random>
#include <math.h>

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



MotionModelOdomMap::MotionModelOdomMap(const int8_t *map, int width, int height, double resolution,float a1, float a2, float a3, float a4) :SampleMotionModel(a1,a2,a3,a4),width_(width), height_(height), resolution_(resolution){

    map_ = (int8_t *) calloc(width*height,sizeof(int8_t));

    for(int i = 0; i < width*height; i++){
        map_[i] = map[i];
    }

}

MotionModelOdomMap::~MotionModelOdomMap(){
    free(map_);
}

Particle MotionModelOdom::sample(const pose_t &curr_odom, const pose_t &prev_odom, const pose_t &allowed_pose){

    pose_t new_pose = {};
    //ist das odometry oder ableitung von odometry
    double dy = curr_odom.y - prev_odom.y;
    double dx = curr_odom.x - prev_odom.x;
    double d_rot_1 = atan2(dy,dx) - prev_odom.theta;
    double d_rot_2 = curr_odom.theta - prev_odom.theta - d_rot_1;
    double d_trans = sqrt(dy*dy+dx*dx);

    double d_rot_prime_1 = d_rot_1 - sample_normal_dist(a1_*d_rot_1+a2_*d_trans);
    double d_rot_prime_2 = d_rot_2 - sample_normal_dist(a1_*d_rot_2+a2_*d_trans);
    double d_trans_prime = d_trans - sample_normal_dist(a3_*d_trans+a4_*(d_rot_1+d_rot_2));

    new_pose.x = allowed_pose.x + d_trans_prime*cos(allowed_pose.theta+d_rot_prime_1);
    new_pose.y = allowed_pose.y + d_trans_prime*sin(allowed_pose.theta+d_rot_prime_1);

    double theta = allowed_pose.theta+d_rot_prime_1+d_rot_prime_2;

    /* make sure theta is in interval [0 2pi]*/
    if (theta < 0.) theta += 2*M_PI;
    else if (theta > 2*M_PI) theta -= 2*M_PI;

    new_pose.theta = theta;

    return Particle(new_pose,0.);
}

Particle MotionModelOdomMap::sample(const pose_t &curr_odom, const pose_t &prev_odom, const pose_t &allowed_pose){

    pose_t prev_odom_pose = allowed_pose;
    double prob = 0.;
    int iter = 0;
    MotionModelOdom odom;

    while(prob <= 0.){


        Particle new_pose = odom.sample(curr_odom, prev_odom, prev_odom_pose);

        /* get map indices */
        int32_t col = new_pose.pose.x/resolution_;
        int32_t row = new_pose.pose.y/resolution_;

        int32_t index = width_*row + col;

        // ROS_INFO_STREAM("[" << row << " " << col << "]");

        if(row < 0 || col < 0 || row > height_ || col > width_ || iter >= 10){
            return Particle(new_pose.pose,0.);
        }

        /* calculate propability */
        // prob = 1. - (index < 0 || index > width_*height_  ?  100. : double(map_[index])) / 100.;
        prob = 1. - ((double) map_[index]) / 100.;
        prev_odom_pose = new_pose.pose;

        iter++;
        //ROS_INFO_STREAM("INDEX " << index);

    }

    return Particle(prev_odom_pose,prob);
}

