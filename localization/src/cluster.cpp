#include "../include/cluster/cluster.h"

Cluster::Cluster(){
    srand(time(NULL));
}

ParticleList Cluster::get_mostlikli_pos(ParticleList &list){
    //start with k = 1
    double k = 1;
    double error = 0;
    //center is mean
    ParticleList centerList;
    Particle center = get_mean(list);
    error = get_error(list, center);
    centerList.append(center);


    //start clustering
    std::vector<ParticleList> cluster;

    //cluster into 2
    k = 2;
    error = k_mean(k, list, centerList, cluster);

    //check if cluster are near
    double dist_k_2 = get_dist(centerList.getParticleAt(0), centerList.getParticleAt(1));
    ROS_INFO_STREAM("cluster dist: " << dist_k_2);
    if(dist_k_2<THRESHOLD_DIST_CENTER || isnan(dist_k_2)){
        centerList.flush();
        centerList.append(center);
        centerList.getParticleAt(0).weight = list.get_sum_weights();
        return centerList;
    }

    //assign weight
    for(auto i = 0; i < centerList.getSize(); i++){
        centerList.getParticleAt(i).weight = cluster[i].get_sum_weights();
    }
    ROS_INFO_STREAM("cluster finished: \n" << centerList);
    return centerList;

}

double Cluster::k_mean(int k, ParticleList &list, ParticleList &centerList, std::vector<ParticleList> &cluster, int maxiter){
    bool debug = false;
    if(debug)ROS_INFO_STREAM("Starting k_mean with k: " << k );
    //init
    ParticleList dummyList; //dummy
    std::vector<double> error;
    double sum_error = 0;
    int iter = 0;

    cluster.clear();
    centerList.flush();
    for(int i = 0; i < k; i++){
        cluster.push_back(dummyList);
        error.push_back(0.0);
    }
    for(int i = 0; i < k; i++){
        //generate random index to choose k random Particle for init
        int rand_idx = rand()%list.getSize();
        Particle randParticle = list.getParticleAt(i);
        if(!centerList.isInList(randParticle)){
            randParticle.weight = 0;
            centerList.append(randParticle);
        }else{
            i--;
        }
    }

    prev_iter_error = 0;
    double error_diff = 0;
    for(int iter = 0; iter < maxiter; iter++){
        if(debug)ROS_INFO_STREAM("kmean, start rekursive");
        //assign each particle to one cluster
        if(debug)ROS_INFO_STREAM("kmean, cluster");
        for(auto i = 0; i < list.getSize(); i++){
            Particle currParticle = list.getParticleAt(i);
            double min_error = 1000000;
            double min_k = 0;
            //get closest center
            for(int j = 0; j < k; j++){
                double error = get_dist(currParticle, centerList.getParticleAt(j));
                if(min_error> error){
                    min_k = j;
                    min_error = error;
                }
            }
            cluster[min_k].append(currParticle);
        }

        if(debug)ROS_INFO_STREAM("kmean, update meanvec");
        //update meanvectors
        centerList.flush();
        for(int i = 0; i < k; i++){
            Particle center = get_mean(cluster[i]);
            centerList.append(center);
        }

        if(debug)ROS_INFO_STREAM("kmean, calc error");

        //get error of each cluster
        for(int i = 0; i < k; i++){
            error[i] = get_error(cluster[i], centerList.getParticleAt(i));
            centerList.getParticleAt(i).weight = error[i];
            sum_error += error[i];
        }

        error_diff = fabs(prev_iter_error-sum_error);
        prev_iter_error = sum_error;
        if(debug)ROS_INFO_STREAM("K-mean- iter: " << iter << " error_diff: " << error_diff << " sum_error:" << sum_error);
        if(!(error_diff > THRESHOLD_DIVERGED && sum_error > THRESHOLD_ERROR)){
            if(debug)ROS_INFO_STREAM("kmean, finished befor maxiter");
        }
    }

    if(debug)ROS_INFO_STREAM("kmean finished!");
    return sum_error;
}

Particle Cluster::get_mean(ParticleList &list){
    Particle center;
    center.pose.x = 0;
    center.pose.y = 0;
    center.pose.theta = 0;
    //count pos theta and neg seperatly (-180,180) = 0 NOT 180 what is wrong
    double theta_pos = 0;
    double theta_neg = 0;
    double weight_pos = 0;
    double weight_neg = 0;
    double total_weight = 0;
    for(auto i = 0; i < list.getSize(); i++){
        Particle curr = list.getParticleAt(i);
        center.pose.x += curr.pose.x * curr.weight;
        center.pose.y += curr.pose.y * curr.weight;
        curr.pose.theta = angles::normalize_angle(curr.pose.theta);
        if(curr.pose.theta>=0){
            theta_pos += curr.pose.theta * curr.weight;
            weight_pos += curr.weight;
        }else{
            theta_neg += curr.pose.theta * curr.weight;
            weight_neg += curr.weight;
        }
        //center.pose.theta += curr.pose.theta * curr.weight;
        total_weight += curr.weight;
    }
    if(weight_pos != 0)
        theta_pos /= weight_pos;
    if(weight_neg != 0)
        theta_neg /= weight_neg;

    double diff_theta = angles::shortest_angular_distance(theta_pos, theta_neg);

    center.pose.x /= total_weight;
    center.pose.y /= total_weight;
    center.pose.theta = theta_pos + diff_theta*weight_neg/(weight_neg+weight_pos);
    center.weight = total_weight;
    return center;
}

double Cluster::get_dist(const Particle &p1, const Particle &p2){
    double diff_x = p1.pose.x - p2.pose.x;
    double diff_y = p1.pose.y - p2.pose.y;
    double diff_theta = p1.pose.theta - p2.pose.theta;

    while(diff_theta>M_PI){
        diff_theta -= 2*M_PI;
    }
    while(diff_theta<-M_PI){
        diff_theta += 2*M_PI;
    }

    return sqrt(pow(diff_x, 2)+pow(diff_y,2));//+pow(diff_theta,2));
}

double Cluster::get_error(ParticleList &list,const Particle &center){
    double error = 0;

    for(auto i = 0; i < list.getSize(); i++){
        Particle curr = list.getParticleAt(i);
        error += curr.weight*get_dist(curr, center);
    }
    return error;
}
