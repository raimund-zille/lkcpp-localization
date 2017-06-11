#include "localization/localization.h"
#include <math.h>
#include <iostream>

#if 0

void augmentedMCL(const std::vector<particle> &particle_old, std::vector<particle> &particle_new, const pose_t &odom_old, const pose_t &odom_new, const std::vector<feature> &features, const std::vector<landmark> &landmarks ){
    size_t m;
    turtlebot_odom_params p; //bestücke p
    p.alpha_1 = 0.1;
    p.alpha_2 = 0.1;
    p.alpha_3 = 0.1;
    p.alpha_4 = 0.1;

    std::vector<particle> particle_updated;
    for(m=0; m < particle_old.size(); m++){
        particle dummyParticle;
        particle_updated.push_back(dummyParticle);
        //update position
        particle_updated[m].pos = sample_motion_model_odometry(odom_new, odom_old, particle_old[m].pos, p);

        //set weight to 0
        particle_updated[m].weight = 0;

        //add for each feature the highest weight of a corresponding landmark to the weight
        size_t i;
        for(i = 0; i < features.size(); i++){
            if(features[i].knownCorr == -1){
                particle_updated[m].weight += landmarkModelUnknownCorr(features[i], particle_new[m].pos, landmarks);
            }else{
                particle_updated[m].weight += landmarkModelKnownCorr(features[i], particle_new[m].pos, landmarks[features[i].knownCorr]);
            }

        }
    }

    //TODO: discard all paritcles with weight < threshhold
    //      create new random particles
    particle_new = particle_updated; //TODO

}

pose_t sample_motion_model_odometry(const pose_t &odom_pose, const pose_t &prev_odom_pose, const pose_t &real_pose, const turtlebot_odom_params &p){

    pose_t new_pose = {};

    double dy = odom_pose.y - prev_odom_pose.y;
    double dx = odom_pose.x - prev_odom_pose.x;
    double d_rot_1 = atan2(dy,dx) - prev_odom_pose.theta;
    double d_rot_2 = odom_pose.theta - prev_odom_pose.theta - d_rot_1;
    double d_trans = sqrt(dy*dy+dx*dx);

    double d_rot_prime_1 = d_rot_1 - sample_normal_dist(p.alpha_1*d_rot_1+p.alpha_2*d_trans);
    double d_rot_prime_2 = d_rot_2 - sample_normal_dist(p.alpha_1*d_rot_2+p.alpha_2*d_trans);
    double d_trans_prime = d_trans - sample_normal_dist(p.alpha_3*d_trans+p.alpha_4*(d_rot_1+d_rot_2));

    new_pose.x = real_pose.x + d_trans_prime*cos(real_pose.theta+d_rot_prime_1);
    new_pose.y = real_pose.y + d_trans_prime*sin(real_pose.theta+d_rot_prime_1);
    new_pose.theta = real_pose.theta+d_rot_prime_1+d_rot_prime_2;

    return new_pose;
}

double landmarkModelUnknownCorr(const feature &feature, const pose_t &pos, const std::vector<landmark> &landmarks){
    double max_weight = 0;
    size_t j;
    for(j = 0; j < landmarks.size(); j++){
        double curr_weight = landmarkModelKnownCorr(feature, pos, landmarks[j]);
        if(curr_weight>max_weight){
            max_weight = curr_weight;
        }
    }
    return max_weight;
}

double landmarkModelKnownCorr(const feature &feature, const pose_t &pos, const landmark &corrLandmark){

    double r_des = 0;
    double phi_des = 0;

    double x_diff = corrLandmark.x - pos.x;
    double y_diff = corrLandmark.y - pos.y;

    r_des = sqrt(pow(x_diff,2)+pow(y_diff,2));
    phi_des = atan2(y_diff, x_diff);

    double prob = getProbGauss((feature.r-r_des), STDDEV_R)*getProbGauss((feature.phi-phi_des),STDDEV_PHI)*getProbGauss((feature.specifier - corrLandmark.specifier), STDDEV_IDF);


    return prob;
}
#endif
