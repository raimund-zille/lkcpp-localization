#ifndef AMCL_H
#define AMCL_H

/* ROS header files */
#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>

/* C++ header files */
#include <vector>
#include <list>
#include <memory>

/* AMCL specific files */
#include "particle.h"
#include "measurement_model.h"
#include "motion_model.h"

/* Augmented MCL decay rates */
#define RECOVERY_ALPHA_FAST   0.1
#define RECOVERY_ALPHA_SLOW 0.001

class AMCL{

public:

    /**
     * @brief AMCL Constructor
     * @param map_width: width of map/playing field
     * @param map_height: heigth of map/playing field
     * @param num_particle: number of particles to generate
     * @param map: Optional pointer to map. If no map is provided this pointer is set to "nullptr"
     * @param resolution: Optional map resolution ~> if map is provided it's required for particle generation
     */
    AMCL(double map_width, double map_height, int num_particle = 500, const int8_t *map = nullptr, float resolution = .05);

    ~AMCL();

    /**
     * @brief augmentedMCL algorithm from "Probabilistic Robotics". Localize robot in a given map
     * @param odom: Odometry data of turtlebot
     * @param motion: Motion model: cf. "motion_model.h" for detailed description
     * @param measure: Measurement model: cf "measurement_model.h" for detailed description
     * @param f: feature
     * @param out: list of poses
     */
    void augmentedMCL(const pose_t &odom, SampleMotionModel &motion, MeasurementModel &measure, const std::vector<feature> &f, ParticleList &out);

    ParticleList getList();

    /**
     * @brief distributeAroundPosition distributes all Particles with gaussian noise around the estimatedPos
     * @param estimatedPos Position around which all Particles will be distributed
     */
    void distributeAroundPosition(pose_t estimatedPos);
    void set_num_particles(int num);
private:

    double width_, heigth_; ///< width and height of map/field
    uint num_particle_;  ///< number of particles to generate
    float resolution_;   ///< map resolution

    int8_t *map_;        ///< map (occupancy grid)
    ParticleList list_;  ///< list of particles

    /**
     * @brief distributeUniformly: Generate a set of uniform distributed particles
     */
    void distributeUniformly(double width, double heigth);

    /**
     * @brief add_gaussian_noise generates gaussian noise
     * @param mean mean of gaussian noise
     * @param stdev stddev of gaussian noise
     * @return gaussian noise value
     */
    double add_gaussian_noise(double mean, double stdev);

    /**
     * @brief getWeightedDist selects particles with a weight greater than 0 and creates a cumulative sum, which is
     * used for the resampling process
     * @param in: current particle list
     * @param out: list of particles with weight greater than 0
     * @param CumulativeSum: vector of cumulative sums
     */
    void getWeightedDist(ParticleList &in, ParticleList &out, std::vector<double> &CumulativeSum);

    /**
     * @brief addRandomParticle: add random particles to the list of particles "list_"
     * @param num: number of particles which should be added
     * @return last generated particle
     */
    Particle addRandomParticle(int num, double width, double heigth);

    /**
     * @brief pickParticle picks a particle considering the importance/weight of particles in "list_"
     * @param Preselection: set of particles that is used for choosing a particle
     * @param CumulativeSum: cumulative sum that is used for choosing a particle
     * @return selected particle
     */
    Particle pickParticle(ParticleList &Preselection, std::vector<double> &CumulativeSum);
    
};

#endif
