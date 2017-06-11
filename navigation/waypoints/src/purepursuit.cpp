#include <algorithm>
#include <cmath>
/* costum header files */
#include "purepursuit.h"
#include "PID.h"

using namespace std;

control PurePursuit::get_control(const pose_t &pos, const pose_t &goal){

    control ctrl;

    /* transform goal to robot frame */


    double xdiff = goal.x - pos.x;
    double ydiff = goal.y - pos.y;

    diff_ = Vector2D(xdiff,ydiff);
    Vector2D dir(pos.theta,RAD);

    double x = xdiff*cos(pos.theta) + ydiff*sin(pos.theta);
    // double y = -xdiff*sin(pos.theta) + ydiff*cos(pos.theta);

    /* robot is not headed to goal or angle is too big */
    double theta = dir.rad(diff_);

    ROS_INFO_STREAM("ANGLE DIFF (DEG): " << theta/M_PI * 180.);
    if (fabs(theta) >= 75./180 * M_PI && fabs(theta) < 90./180.*M_PI){
        ctrl.w = (theta < 0 ? -TURTLEBOT_ANG_MAX_RPS : TURTLEBOT_ANG_MAX_RPS)*0.25;
        ctrl.v = 0;
        return ctrl;
    }
    else if (fabs(theta ) >= 90./180.*M_PI){
        ctrl.w = (theta < 0 ? -TURTLEBOT_ANG_MAX_RPS : TURTLEBOT_ANG_MAX_RPS)*0.5;
        ctrl.v = 0;
        return ctrl;
    }



    /*
    if (dir.dot(diff_) < 0){

        ctrl.v = 0;
        ctrl.w = saturation(theta,-TURTLEBOT_ANG_MAX_RPS,TURTLEBOT_ANG_MAX_RPS);
        return ctrl;
    }
    */


    //double l_2 = diff_.dot(diff_);

    /* wanted current speed */
    // double speed = saturation(.3 * diff_.magnitute() - .05, .1, TURTLEBOT_VEL_MAX_MPS);
    double speed = .25;
    double l = 1.;
    double gamma = 2*x/(l*l);

    ctrl.v = speed;
    ctrl.w = gamma*speed;

    return ctrl;




}

bool PurePursuit::reached_goal(double thresh){
    return diff_.magnitute() < thresh;
}
