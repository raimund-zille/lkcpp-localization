#ifndef PARTICLE_H
#define PARTICLE_H

#include "ros/ros.h"
#include <vector>
#include <ostream>
#include <angles/angles.h>

struct pose_t {       /// pose of robot (coordinate and angular direction)

    double x = 0;     /// x coordinate
    double y = 0;     /// y coordinate
    double theta = 0; /// angular direction

    bool operator==(const pose_t &other){
        return (x == other.x) && (y == other.y) && (theta == other.theta);
    }

    bool operator!=(const pose_t &other){
        return (*this == other);
    }

    friend std::ostream &operator<<(std::ostream &out, const pose_t &pose){
        out << "[" << pose.x << " " << pose.y << " " << angles::to_degrees(pose.theta) << "]" << std::endl;
        return out;
    }
};

class Particle{

public:

    Particle():pose(pose_t()), weight(0){}
    Particle(const pose_t &p, double w):pose(p),weight(w){}

    pose_t pose;
    double weight;

    Particle& operator=(const Particle &other){
        pose = other.pose;
        weight = other.weight;
        return *this;
    }

    bool operator ==(const Particle &other){
        return pose == other.pose && weight == other.weight;
    }

    bool operator !=(const Particle &other){
        return (*this == other);
    }

    bool operator <(const Particle &other) const{
        return weight < other.weight;
    }

    friend std::ostream& operator<< (std::ostream &out, const Particle &p);

};

class ParticleList{

public:    

    ParticleList();
    ParticleList(const ParticleList &pList);

    /**
     * @brief append a particle to current list
     * @param p: particle which should be included to list
     */
    void append(const Particle &p);

    /**
     * @brief remove a particle at index "idx"
     * @param idx: index of particle which should be removed
     */
    void remove(int idx);

    /**
     * @brief flush whole content of particle list
     */
    void flush();

    /**
     * @brief get_sum_weights: get sum of all weights in list
     * @return sum of weights
     */
    double get_sum_weights() const;

    /**
     * @brief normalize all weights of list, so the total sum equals 1
     */
    void normalize(void);

    /**
     * @brief sort particle list in ascending order considering the weight of the particles
     */
    void sort(void);

    /**
     * @brief isInList: Check if a particle is allready in list
     * @param p: Particle
     * @return true if "p" is allready in list false otherwise
     */ //static std::uniform_real_distribution<double> distribution(0,2*M_PI);
    bool isInList(const Particle &p);

    /**
     * @brief getParticleAt: Return particle at corresponding index
     * @param idx
     * @return Particle at index "idx"
     */
    Particle &getParticleAt(int idx);

    /**
     * @brief getSize: Size of particle list
     * @return size of particle list
     */
    size_t getSize() const;

    ParticleList &operator=(const ParticleList &pList);
    friend std::ostream &operator<<(std::ostream &out, ParticleList &list);

private:

    std::vector<Particle> particle_list_;
    double sum_weights_;

    
};

#endif
