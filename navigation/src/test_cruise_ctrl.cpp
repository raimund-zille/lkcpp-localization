#include <ros/ros.h>
#include <angles/angles.h>
#include <list>

#include "cruisecontrol.h"

using namespace std;
using namespace angles;

struct DistAngle{
    double dist;
    double rel_angle;
};

static void addToList(double dist, double angle, std::list<DistAngle> &out){
    DistAngle d;
    d.dist = dist;
    d.rel_angle = angle;
    out.push_back(d);
}

/*
template <typename T> addToList(const typename T& in, std::list &out){
    out.
}
*/

int main(int argc, char **argv){

    ros::init(argc,argv,"test_cruise_ctrl_node");



    std::list<double> angles;
    std::list<DistAngle> dist_angle;
    std::list<Vector2D> odom_pos;
    bool reached_goal = true;
    double current_angle = 0;

    //for (double angle = 30.; angle < 180.; angle += 30.){
        //angles.push_back(from_degrees(angle));
    //}

    /*
    angles.push_back(from_degrees(-180.));
    angles.push_back(from_degrees(45.));
    angles.push_back(from_degrees(-45.));
    angles.push_back(from_degrees(359.));
    angles.push_back(from_degrees(-359.));
    angles.push_back(from_degrees(-420.));
    */

    /*
    addToList(1.0,from_degrees(90.),dist_angle);
    addToList(1.0,from_degrees(90.),dist_angle);
    addToList(1.0,from_degrees(90.),dist_angle);
    addToList(1.0,from_degrees(90.),dist_angle);
    */

    odom_pos.push_back(Vector2D(1,0));
    odom_pos.push_back(Vector2D(2,0));
    ros::NodeHandle nh;
    CruiseControl cruise(nh);
    //cruise.turn(from_degrees(2.),.1);
    //cruise.start();
    //cruise.stop();
    ros::Rate r(20);
    while(ros::ok()){

        if (reached_goal && !odom_pos.empty()){
            // double current_angle = *angles.begin();
            //DistAngle current = *dist_angle.begin();
            Vector2D odom_goal = *odom_pos.begin();
            if(cruise.moveToOdomPos(odom_goal)){
                //ROS_WARN_STREAM("REACHED GOAL, NEXT TURNING ANGLE/DIST: (" << to_degrees(current.rel_angle) << " " << current.dist << ")");
                ROS_WARN_STREAM("REACHED GOAL, NEXT: " << odom_goal);
                odom_pos.pop_front();
                reached_goal = false;
            }
            // angles.pop_front();

            //ROS_WARN_STREAM("REACHED ANGLE, NEXT TURNING ANGLE: " << to_degrees(current_angle));

            //cruise.turn(current_angle,from_degrees(1.));

                       //cruise.stop();
            //ros::Rate(1.).sleep();
            //cruise.start();
        }

        reached_goal = cruise.reachedGoal();

        ros::spinOnce();
        r.sleep();
    }

}
