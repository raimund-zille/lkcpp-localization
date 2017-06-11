#include "particle.h"
#include <iostream>
#include <ros/ros.h>

ParticleList::ParticleList():sum_weights_(0) {}

ParticleList::ParticleList(const ParticleList &pList){
    for (auto p = pList.particle_list_.begin(); p != pList.particle_list_.end(); p++){
        append(*p);
    }
    this->sum_weights_ = pList.sum_weights_;
}

void ParticleList::append(const Particle &p){
    particle_list_.push_back(p);
    sum_weights_ += p.weight;
}

void ParticleList::remove(int idx){

    auto remove_element = particle_list_.begin();

    /* update iterator to matching element */
    advance(remove_element,idx);

    /* update current sum of list */

    sum_weights_ -= remove_element->weight;
    /* remove element from list */
    remove_element = particle_list_.erase(remove_element);

}

void ParticleList::flush(){
    particle_list_.clear();
    sum_weights_ = 0;
}

void ParticleList::normalize(){
    double sum = get_sum_weights();
    for (auto p = particle_list_.begin(); p != particle_list_.end(); p++){
        p->weight /= sum;
    }
}

double ParticleList::get_sum_weights() const{
    /*
    double sum = 0.;
    for (auto p = particle_list_.begin(); p != particle_list_.end(); p++){
        sum += p->weight;
    }
    return sum;
    */
    return sum_weights_;
}

bool ParticleList::isInList(const Particle &particle){
    for (auto p = particle_list_.begin(); p != particle_list_.end(); p++){
        if (*p == particle) return true;
    }
    return false;
}

size_t ParticleList::getSize() const {
    return particle_list_.size();
}

Particle &ParticleList::getParticleAt(int idx){
    /* exit programm if index is out of bounds */
    assert(!(0 < idx && idx > particle_list_.size()));
    return particle_list_[idx];
}

void ParticleList::sort(){
    std::sort(particle_list_.begin(),particle_list_.end());
}


ParticleList &ParticleList::operator =(const ParticleList &pList){

    /* flush contents of list and append new elements */
    flush();
    for (auto p = pList.particle_list_.begin(); p!= pList.particle_list_.end(); p++){
        append(*p);
    }
    sum_weights_ = pList.get_sum_weights();
}

std::ostream& operator<< (std::ostream &out, const Particle &p){
    out << "Particle [" << p.pose.x << " " << p.pose.y << " " << p.pose.theta << " " << p.weight << "]";
    return out;
}

std::ostream& operator <<(std::ostream &out, ParticleList &list){

    for(auto i = 0; i < list.getSize(); i++){

        out << list.getParticleAt(i) << std::endl;
    }

    return out;
}
