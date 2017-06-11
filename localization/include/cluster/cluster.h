#ifndef CLUSTER_H
#define CLUSTER_H
#include "math.h"
#include "ros/ros.h"
#include "../amcl/inc/particle.h"
#include <vector>
#include "random"
#include <angles/angles.h>
#include "tf/tf.h"

#define THRESHOLD_ERROR 10
#define THRESHOLD_DIVERGED 0.001
#define THRESHOLD_DIST_CENTER 0.9

class Cluster{
public:
    /**
     * @brief Cluster default constructor
     */
    Cluster();

    /**
     * @brief get_mostlikli_pos returns 1 or 2 positions which are most likley
     * @param list given positions
     * @return list (1/2) of the position
     */
    ParticleList get_mostlikli_pos(ParticleList &list);

    /**
     * @brief get_mean returns mean
     * @param list Particleslist which shall be meaned
     * @return returns mean particle
     */
    Particle get_mean(ParticleList &list);

    /**
     * @brief get_error returns error to center
     * @param list list on which error should be calculated
     * @param center center to which the error shall be calculated
     * @return error
     */
    double get_error(ParticleList &list, const Particle &center);

    /**
     * @brief get_dist calculates dist between to particles without using the angle
     * @param p1 Particle 1
     * @param p2 Particle 2
     * @return dist between the particles
     */
    double get_dist(const Particle &p1, const Particle &p2);

    /**
     * @brief k_mean clusters particles with k-mean
     * @param k count of centers
     * @param list particles
     * @param centerList there will be the k centers
     * @param cluster k particlelists for each cluster
     * @param maxiter maximum iteration it should try to find the centers
     * @return returns cummulativ error of all centers
     */
    double k_mean(int k, ParticleList &list, ParticleList &centerList, std::vector<ParticleList> &cluster, int maxiter = 10);

private:
    double prev_iter_error;
};



#endif
