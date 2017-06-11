#include "amcl.h"

#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <assert.h>
#include <typeinfo>

#include <ros/ros.h>

/*
AMCL::AMCL(const int8_t *map, int map_width, int map_height, float resolution, uint num_particle, double a, double b,
           float a1, float a2, float a3, float a4){

    srand48(time(NULL));
    srand(time(NULL));
    width_= map_width;
    heigth_ = map_height;
    resolution_ = resolution;
    num_particle_ = num_particle;
    a1_= a1;
    a2_= a2;
    a3_= a3;
    a4_= a4;
    distA_ = a;
    distB_ = b;
    map_ = (int8_t*) calloc(map_width*map_height,sizeof(int8_t));

    for (int i = 0; i < width_*heigth_; i++){
        map_[i] = map[i];
    }

    distributeUniformly();
}
*/

AMCL::AMCL(double map_width, double map_height, int num_particle, const int8_t *map, float resolution){


    srand48(time(NULL));
    srand(time(NULL));
    width_= map_width;
    heigth_ = map_height;
    resolution_ = resolution;
    num_particle_ = num_particle;

    if (map != nullptr){

        map_ = (int8_t*) calloc(map_width*map_height,sizeof(int8_t));

        for (int i = 0; i < width_*heigth_; i++){
            map_[i] = map[i];
        }

    }
    else map_ = nullptr;

    ROS_INFO_STREAM("WIDTH: " << width_ << " HEIGHT: " << heigth_);

    distributeUniformly(map_width/3, map_height);
}

AMCL::~AMCL(){
    if (map_ != nullptr) free(map_);
}

ParticleList AMCL::getList(){
    return list_;
}

Particle AMCL::addRandomParticle(int num, double width, double heigth){

    int size  = width_*heigth_;
    int generated_particles = 0;
    Particle current;

    static std::random_device rd;
    static std::mt19937 generator(rd());
    //std::default_random_engine generator;
    static std::uniform_real_distribution<double> distributionTheta(0,2*M_PI);
    static std::uniform_real_distribution<double> distributionX(0,width);
    static std::uniform_real_distribution<double> distributionY(0,heigth);

    /* select particles from allowed coordinates */
    while(generated_particles < num){

        if (map_ != nullptr){

            int idx = rand() % size;
            int8_t occupancy = map_[idx];

            /* if cell is occupied or region is unknown don't generate a particle there */
            if (occupancy == 100 || occupancy == -1) continue;

            /* assign particle position and angular direction */

            int row = idx / width_;
            int col = idx - row*width_;
            current.pose.x = col*resolution_;
            current.pose.y = row*resolution_;

        }
        else{

            current.pose.x = distributionX(generator);
            current.pose.y = distributionY(generator);
            current.pose.theta = distributionTheta(generator);//2*M_PI*drand48();

        }

        current.weight = 0.01;


        /* if this particle happened to be in list then don't append it */
        //if (list_.isInList(current)) continue; auskommentiert da unnoetiger rechenaufwand

        list_.append(current);
        generated_particles++;
    }


    /* return last particle which was generated */
    return current;
}

double AMCL::add_gaussian_noise(double mean, double stdev){

    static std::random_device rd;
    static std::mt19937 generator(rd());

    std::normal_distribution<double> gauss(mean,stdev);

    return gauss(generator);

}

void AMCL::distributeAroundPosition(pose_t estimatedPos){
    list_.flush();
    ROS_INFO_STREAM("distribute around: " << estimatedPos.x << "/" << estimatedPos.y << "/" << estimatedPos.theta);

    for(auto i = 0; i < num_particle_; i++){
        pose_t pos;
        pos.x = add_gaussian_noise(estimatedPos.x, 0.2); //stdabweichung von 0.5 meter
        pos.y = add_gaussian_noise(estimatedPos.y, 0.2);
        pos.theta = add_gaussian_noise(estimatedPos.theta, M_PI_4/2); //stdabweichung von 45°
        Particle curr;
        curr.pose = pos;
        curr.weight = 0.1;
        list_.append(curr);
    }
    list_.normalize();
}

void AMCL::distributeUniformly(double width, double height){

    /* check if particle number is smaller equal to map size*/
    // assert(!(num_particle_ > width_*heigth_));

    /* flush current particle list */
    list_.flush();
    addRandomParticle(num_particle_, width, height);
    /* normalize particle weights */
    list_.normalize();

    /*
    for (int i = 0; i < list_.getSize(); i++)
          ROS_INFO_STREAM(list_.getParticleAt(i));
    */
}

void AMCL::getWeightedDist(ParticleList &in, ParticleList &out, std::vector<double> &CumulativeSum){

    out.flush();
    CumulativeSum.clear();
    ROS_INFO_STREAM("GET WEIGHTED DIST");
    /* compute comulative sum, which is used for "drawing" the new set of particles */

    double sum = 0;
    for (auto i = 0; i < in.getSize(); i++){

        Particle p = in.getParticleAt(i);
        double w = p.weight;

        if (w > 0.){
            sum += w;
            CumulativeSum.push_back(sum);
            out.append(p);
            // ROS_INFO_STREAM("PRESELECTED: " << p);
        }

    }

}

void AMCL::set_num_particles(int num){
    if(num>0)
        num_particle_ = num;
}

Particle AMCL::pickParticle(ParticleList &Preselection, std::vector<double> &CumulativeSum){

    /* Find index which is in a specific range. The cumulative Sum contains values like [0.1,0.2,1.0].
    * If the selected random number is e.g. 0.3 the interval [0.2,1.0] is selected.
    * The index of the range corresponds to the particle, which is chosen for the possible poses of the robot.
    * For further details please cf.:
    * http://robotics.stackexchange.com/questions/479/particle-filters-how-to-do-resampling
    */

    int index = 0;
    double num = drand48();
    for (int i = 1; i < CumulativeSum.size(); i++){
        if (CumulativeSum[i-1] <= num && num <= CumulativeSum[i]){
            //  ROS_INFO_STREAM("index: " << i << " prob: " << num);
            index =  i;
        }
    }
    // ROS_INFO_STREAM("PICK PARTICLE AT: " << index << " SIZE PARTICLE LIST: " << Preselection.getSize());
    Particle picked = Preselection.getParticleAt(index);
    picked.pose.x = picked.pose.x + add_gaussian_noise(0, 0.1);
    picked.pose.y = picked.pose.y + add_gaussian_noise(0, 0.1);
    picked.pose.theta = picked.pose.theta + add_gaussian_noise(0, 2*3.1415/180);
    return picked;
}


void AMCL::augmentedMCL(const pose_t &odom, SampleMotionModel &motion, MeasurementModel &measure, const std::vector<feature>& f, ParticleList &out){

    static double w_fast = 0., w_slow = 0.;
    static pose_t prev_odom = {};
    ParticleList new_particles;
    bool isMotionModelOdomMap = true;

    /* out of bound */
    bool oob_x = false;
    bool oob_y = false;

    /* make sure out is empty */
    out.flush();

    /* check if motion is of type MotionModelOdomMap */
    try  {dynamic_cast<MotionModelOdomMap&>(motion);}
    catch(const std::bad_cast& e){

        /* motion model is not of type MotionModelOdomMap */
        isMotionModelOdomMap = false;
        std::cout << e.what() << std::endl;
    }


    /* iterate through list of particles */
    for (int i = 0; i < list_.getSize(); i++){

        /* get new motion sample and assign new weight to it */
        Particle sample = motion.sample(odom,prev_odom,list_.getParticleAt(i).pose);
        sample.weight = list_.getParticleAt(i).weight;

        /*
        if(!isMotionModelOdomMap && !f.empty()){
            sample.weight = list_.getParticleAt(i).weight;
        }
        */

        /* if motion is not of type MotionModelOdomMap get weight from measurement model */
        if (!isMotionModelOdomMap && !f.empty()){
            sample.weight = measure.get_weight(sample.pose, f);
        }

        /* reject samples that are totally out of bound */
        if(!isMotionModelOdomMap){
            //oob_x = sample.pose.x < 0 || sample.pose.x > width_;
            //oob_y = sample.pose.y < 0 || sample.pose.y > heigth_;
        }


        //if (oob_x || oob_y) continue;/* sample.weight = 0.;*/
        new_particles.append(sample);

    }

    /* if no features were seen just take new motion samples and return */
    if (f.empty()){
        prev_odom = odom;
        list_ = new_particles;
        for (auto i = 0; i < new_particles.getSize(); i++)
            out.append(new_particles.getParticleAt(i));
        return;
    }

    // double curr_sum = 0;
    double sum = new_particles.get_sum_weights();
    if (sum == 0.) {
        distributeUniformly(width_, heigth_);
        new_particles = list_;
        sum = new_particles.get_sum_weights();
    }

    new_particles.normalize();

    double w_avg = sum/num_particle_;

    w_slow += RECOVERY_ALPHA_SLOW*(w_avg - w_slow);
    w_fast += RECOVERY_ALPHA_FAST*(w_avg - w_fast);

    ROS_INFO_STREAM("W_SLOW: " << w_slow << " W_FAST: " << w_fast);

    ParticleList PreSelection;
    std::vector<double> CumulativeSum;

    /* sort particles ~> ascending weights */
    new_particles.sort();

    /* discard particles with zero weight and generate a cumulative sum which is used for the resampling process */
    getWeightedDist(new_particles,PreSelection,CumulativeSum);

    /* probability to add random particle */
    double random_prob = std::max(0.,1.-(w_fast/w_slow));

    int random_particle = 0;
    int picked_particle = 0;
    list_.flush();
    /* chose new set of particles */
    for (int i = 0; i < num_particle_; i++){

        Particle p;
        // std::string info;
        p = drand48() < random_prob ? (random_particle++, addRandomParticle(1, width_, heigth_)) : (picked_particle++, pickParticle(PreSelection,CumulativeSum));
        //ROS_INFO_STREAM(info << p);

        out.append(p);
        list_.append(p);


    }

    ROS_INFO_STREAM("RANDOM: " << random_particle << " PICKED: " << picked_particle);

    // ROS_INFO_STREAM(list_);
    prev_odom = odom;
}
