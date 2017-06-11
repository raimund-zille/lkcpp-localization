#ifndef A_STAR
#define A_STAR

/* ros header files */
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

/* DEBUG
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
*/

/* c++ header files */
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>
#include <ostream>

#include <boost/heap/fibonacci_heap.hpp>

struct pos_t{

    pos_t(int x = 0, int y = 0){
        this->x = x;
        this->y = y;
    }

    int x, y;

    pos_t &operator=(const pos_t &other){
        x = other.x;
        y = other.y;
    }

    bool operator==(const pos_t &other) const{
        return ((x == other.x) && (y == other.y));
    }

    pos_t &operator-=(const pos_t &other){
        x -= other.x;
        y -= other.y;
        return *this;
    }

    pos_t operator-(const pos_t &other) const{
        pos_t ret(x,y);
        ret -= other;
        return ret;
    }

    pos_t &operator+=(const pos_t &other){
        x += other.x;
        y += other.y;
        return *this;
    }

    pos_t operator+(const pos_t &other) const{
        pos_t ret(x,y);
        ret += other;
        return ret;
    }

    friend std::ostream &operator<<(std::ostream &out, const pos_t &p){
        out << "coordinate: (" << p.x << " " << p.y << ")" << std::endl;
        return out;
    }

};

class Heuristic{

public:

    Heuristic(){}
    virtual double get_priority(const pos_t &p1, const pos_t &p2) const = 0;
};

/**
 * @brief The EuclidDist class is a heuristic, which is used to assign a speficic position a priority.
 */
class ManhattanDist : public Heuristic{

public:

    ManhattanDist():Heuristic(){}
    double get_priority(const pos_t &p1, const pos_t &p2) const{
        pos_t diff = p2 - p1;
        return std::abs(diff.x) + std::abs(diff.y);
    }
};

/**
 * @brief The EuclidDist class is a heuristic, which is used to assign a speficic position a priority.
 * It is the shortest possible distance from start to goal (straight line)
 */
class EuclidDist : public Heuristic{

public:

    EuclidDist() : Heuristic(){}
    double get_priority(const pos_t &p1, const pos_t &p2) const{
        pos_t diff = p2 - p1;
        return std::sqrt(diff.x*diff.x + diff.y*diff.y);
    }
};

/**
 * @brief The ZeroCost assigns a zero priority. A* behaves like Dijkstra
 */
class ZeroCost : public Heuristic{

public:
    ZeroCost() : Heuristic(){}
    double get_priority(const pos_t &p1, const pos_t &p2) const {return 0.;}
};

/**
 * @brief The Pos class is a simple helper class for the astar algorithm to keep track of the costs and moves (positions)
 */
class Pos{

public:

    Pos(){
        this->pos = pos_t(0,0);
        this->cost = 0;
        this->priority = 0;
        this->moves = std::vector<pos_t>();

    }

    Pos(const pos_t &pos, const std::vector<pos_t> &moves ,double cost = 0, double priority = 0){
        this->pos = pos;
        this->cost = cost;
        this->priority = priority;
        this->moves = moves;
    }

    pos_t pos;          ///< current position on map
    double cost;        ///< current cost to travel from start to coordinate "pos"
    double priority;    ///< priority = cost(from start to pos) + heuristic(pos to goal)
    boost::heap::fibonacci_heap<Pos,boost::heap::compare<std::greater<Pos>>>::handle_type handle; ///< boost handle to access elements from priority queue
    std::vector<pos_t> moves; ///< coordinates to get to position "pos"

    bool operator>(const Pos &other) const{
        return priority > other.priority;
    }

    Pos &operator=(const Pos &other){
        pos = other.pos;
        cost = other.cost;
        priority = other.priority;
        moves = other.moves;
    }

    bool operator==(const Pos &other) const{
        return pos == other.pos;
    }

    friend std::ostream &operator<<(std::ostream &out, const Pos &p){
        out << p.pos;
        out << "current cost: " << p.cost << std::endl;
        out << "priority: " << p.priority << std::endl;
    }

};

static EuclidDist euclid; // default A* heuristic

/**
 * @brief The Astar class searches for an optimal path in a ROS occupancy grid
 */
class Astar{

public:

    Astar():map_(nullptr),width_(0),height_(0),resolution_(0){}

    /**
     * @brief set_map passes the current map to the Astar search algorithm
     * @param map: pointer to map (occupancy grid)
     * @param width: map width
     * @param height: map height
     */
    void set_map(const int8_t *map, int width, int height, float resolution);

    /**
     * @brief findPath searches for a path for a given start and goal
     * @param start: start position
     * @param goal: goal position
     * @param out: path from start to goal in form of a vector of positions
     * @param heuristic: heuristic to assign a priority for search
     * @return true if path exists, false otherwise
     */
    bool findPath(const pos_t &start, const pos_t &goal, std::vector<geometry_msgs::PoseStamped>  &out, const Heuristic &heuristic = euclid);

private:

    const int8_t *map_;   ///< pointer to current map
    int width_, height_;  ///< map width and height
    float resolution_;

    /**
     * @brief getSuccessors searches for possible successors for a given position "pos"
     * @param pos: input position
     * @param out: vector of possible successors
     */
    void getSuccessors(const pos_t &pos, std::vector<pos_t> &out);


    /**
     * @brief createPoses of A* route
     * @param in: Path
     * @param out: Poses of Path
     */
    void createPoses(const std::vector<pos_t> &in, std::vector<geometry_msgs::PoseStamped>  &out);

};

#endif /* A_STAR */
