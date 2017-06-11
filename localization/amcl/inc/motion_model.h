#ifndef MOTION_MODE_H
#define MOTION_MODE_H

#include "particle.h"

class SampleMotionModel{
public:

    SampleMotionModel(float a1  = 0.001, float a2 = 0.001 , float a3 = 0.001, float a4 = 0.001) : a1_(a1),a2_(a2),a3_(a3),a4_(a4){}
    virtual Particle sample(const pose_t &curr_odom, const pose_t &prev_odom, const pose_t &allowed_pose) = 0;

protected:
    float a1_, a2_, a3_, a4_;
};

/**
 * @brief The MotionModelOdom class samples a new pose without considering a map. It's only input is odometry data.
 */
class MotionModelOdom : public SampleMotionModel{

public:

    MotionModelOdom(float a1  = 0.001, float a2 = 0.001 , float a3 = 0.001, float a4 = 0.001):SampleMotionModel(a1,a2,a3,a4){}

    /**
     * @brief sample: Sample a new "pose" and return a particle. The weight of the particle is assigned to 0
     * @param curr_odom: Current odometry data of the robot
     * @param prev_odom: Previous odometry data of the robot
     * @param allowed_pose: Allowed pose in therms of the map
     * @return Particle which contains a pose but a "0"-weight
     */
    Particle sample(const pose_t &curr_odom, const pose_t &prev_odom, const pose_t &allowed_pose);
};
/**
 * @brief The MotionModelOdomMap class samples a particle with weight.
 * It is considering a occupancy map to calculate pose and weight.
 */
class MotionModelOdomMap : public SampleMotionModel{

public:

    MotionModelOdomMap(const int8_t *map, int width, int height, double resolution,
                       float a1  = 0.001, float a2 = 0.001 , float a3 = 0.001, float a4 = 0.001
                       );

    ~MotionModelOdomMap();

    /**
     * @brief sample: Sample a new "pose" and return a particle with weight, which considers the maps occupancy.     * @param curr_odom: Current odometry data of the robot
     * @param prev_odom: Previous odometry data of the robot
     * @param allowed_pose: Allowed pose in therms of the map
     * @return Particle with pose and assigned weight
     */
    Particle sample(const pose_t &curr_odom, const pose_t &prev_odom, const pose_t &allowed_pose);

private:

    int8_t *map_;
    int width_, height_;
    double resolution_;
};

#endif
