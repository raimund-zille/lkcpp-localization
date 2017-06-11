/* ROS header files */
#include<tf/tf.h>

/* boost priority queue header */
#include <boost/heap/priority_queue.hpp>


#include "astar.h"

using namespace std;
using namespace tf;
using namespace geometry_msgs;
// using namespace cv;


typedef boost::heap::fibonacci_heap<Pos, boost::heap::compare<greater<Pos>>> fibonacci_heap;
typedef fibonacci_heap::handle_type handle_t;

void Astar::set_map(const int8_t *map, int width, int height, float resolution){

    map_ = map;
    width_ = width;
    height_ = height;
    resolution_ = resolution;

}

/* debug purposes*/
/*
static Vec3b hsv_to_rgb (unsigned char h, unsigned char s, unsigned char v)
{
    unsigned char r,g,b, i, f;
    unsigned int p, q, t;

    if( s == 0 )
    {r = g = b = v; }
    else
    {
        i=h/43;
        f=h%43;
        p = (v * (255 - s))/256;
        q = (v * ((10710 - (s * f))/42))/256;
        t = (v * ((10710 - (s * (42 - f)))/42))/256;

        switch( i )
        {
        case 0:
            r = v; g = t; b = p; break;
        case 1:
            r = q; g = v; b = p; break;
        case 2:
            r = p; g = v; b = t; break;
        case 3:
            r = p; g = q; b = v; break;
        case 4:
            r = t; g = p; b = v; break;
        case 5:
            r = v; g = p; b = q; break;
        }
    }

    return Vec3b(b,g,r);


}
*/

void Astar::getSuccessors(const pos_t &pos, std::vector<pos_t> &out){

    /* make sure out is empty */
    out.clear();

    /* make sure parameters are set and nothing is out of bounds */
    if(map_ == nullptr || width_ <= 0 || height_ <= 0) return;
    if(!(pos.x >= 0 && pos.x < width_ && pos.y >= 0 && pos.y < height_)) return;

    /* pos is in the center of the possible successors */
    /* start with top left corner of pos */
    int row = pos.y - 1;
    int col = pos.x - 1;

    /* at maximum 8 possible successors */
    for (int i = row; i < row + 3; i++){
        for (int j = col; j < col + 3; j++){

            if (j < 0 || j > width_ || i < 0 || i > height_) continue;
            if (j == pos.x && i == pos.y) continue;

            int occupancy = map_[i*width_ + j];
            if (occupancy >= 100 || occupancy < 0) continue;

            pos_t current(j,i);
            out.push_back(current);

        }

    }
}

void Astar::createPoses(const vector<pos_t> &in, std::vector<PoseStamped> &out){

    out.clear();
    pos_t current(0,0);

    for (auto p : in){

        pos_t diff = p - current; // direction

        double yaw = atan2(diff.y,diff.x);

        tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
        PoseStamped pose;

        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "/map";
        memcpy(&pose.pose.orientation,q, sizeof(tf::Quaternion));

        pose.pose.position.x = p.x*resolution_;
        pose.pose.position.y = p.y*resolution_;
        pose.pose.position.z = 0;

        out.push_back(pose);

        current = p;

    }

}

bool Astar::findPath(const pos_t &start, const pos_t &goal, std::vector<PoseStamped> &out, const Heuristic &heuristic){

    /* check if map is a nullptr */
    if(map_ == nullptr) return false;

    /* is the resolution valid? */
    if(resolution_ <= 0){
        ROS_INFO_STREAM("RESOLUTION NOT VALID");
        return false;
    }

    /* boundary check */
    if(!(start.x >= 0 && start.x < width_ && start.y >= 0 && start.y < height_)){
        ROS_INFO_STREAM("START POSITION OUT OF BOUNDS: " << start);
        return false;
    }

    if(!(goal.x >= 0 && goal.x < width_ && goal.y >= 0 && goal.y < height_)){
        ROS_INFO_STREAM("GOAL POSITION OUT OF BOUNDS: " << start);
        return false;
    }

    /* is start or goal occupied ? */
    if (map_[start.y*width_ + height_] == 100 || map_[start.y*width_ + height_] < 0){
        ROS_INFO_STREAM ("NOT A VALID START POSE " << start);
        return false;
    }

    if (map_[goal.y*width_ + height_] == 100 || map_[goal.y*width_ + height_] < 0){
        ROS_INFO_STREAM ("NOT A VALID GOAL POSE " << start);
        return false;
    }

    vector<pos_t> visited;
    vector<handle_t> handles;

    /* loswest priority shall appear on top */
    fibonacci_heap PrioQueue;

    /* setup astar start values */
    Pos start_pos(start,vector<pos_t>(),0,heuristic.get_priority(start,goal));
    boost::heap::fibonacci_heap<Pos,boost::heap::compare<greater<Pos>>>::handle_type handle = PrioQueue.push(start_pos);
    (*handle).handle = handle;
    handles.push_back(handle);

    int expanded_nodes = 0;

    while(!PrioQueue.empty() && ros::ok()){

        /* get pos with hightes priority */
        Pos current_pos = PrioQueue.top();

        /* remove current handle from "handles" so it wont be used again */
        handles.erase(std::remove(handles.begin(), handles.end(), current_pos.handle), handles.end());

        /* remove element from priority queue */
        PrioQueue.pop();

        /* if goal is found terminate */
        if (current_pos.pos == goal){
            //out = current_pos.moves;
            ROS_INFO_STREAM("NODES EXPANDED: " << expanded_nodes);
            createPoses(current_pos.moves,out);
            return true;
        }

        /* keep track of allready visited positions */
        visited.push_back(current_pos.pos);

        vector<pos_t> suc;

        /* get possible successors */
        getSuccessors(current_pos.pos,suc);

        /* expand nodes */
        for (auto pos : suc){

            if (find(visited.begin(), visited.end(),pos) != visited.end()) continue;

            pos_t diff = pos - current_pos.pos;

            /* get the cost to move from one cell to the next cell */
            double cost_diff = sqrt(diff.x*diff.x + diff.y*diff.y);
            double next_cost = current_pos.cost + cost_diff;
            double next_prio = next_cost*map_[pos.y*width_+pos.x] + heuristic.get_priority(pos,goal);

            Pos current = current_pos;
            current.pos = pos;
            current.cost = next_cost;
            current.priority = next_prio;
            current.moves.push_back(pos);

            bool isInQueue = false;
            int idx = 0;
            for (int i = 0; i < handles.size(); i++){
                if((*handles[i]).pos == pos){
                    idx = i;
                    isInQueue = true;
                    break;
                }
            }


            if(!isInQueue){

                handle_t handle = PrioQueue.push(current);
                (*handle).handle = handle;
                handles.push_back(handle);

            }
            else if(next_cost < (*handles[idx]).cost){
                PrioQueue.update(handles[idx],current);
            }

            expanded_nodes++;

        }


    }

    /* the goal could not be found e.g. it was placed in a closed room */
    return false;

}
