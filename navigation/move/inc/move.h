#ifndef MOVE_H
#define MOVE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "PID.h"
#include "vector2d.h"
#include "particle.h"

/* see turtlebot specs  */
#define TURTLEBOT_VEL_MAX_MPS .65
#define TURTLEBOT_ANG_MAX_RPS  M_PI

/* PID values for TURTLEBOT */
#define P_ANGLE (4.)
#define I_ANGLE  0.
#define D_ANGLE  0.//(1./3000.)

#define P_VEL (1.5)
#define I_VEL  0.
#define D_VEL  0.//(1./1000.)

/**
 * @brief PublishCmdVel: Publish linear and angular velocity commands
 * @param angular: angular velocity
 * @param speed: linear velocity
 * @param vel_cmd: ros publisher
 */
void PublishCmdVel(double angular, double speed, const ros::Publisher &vel_cmd);

/**
 * @brief The Move class is an abstract base class. Classes that derive from it are forced to implement the pure virtual methods
 */
class Move{

public:

    Move(){}
    virtual void move(double dt, const ros::Publisher &vel_cmd) = 0;
    virtual bool reachedGoal() = 0;
    void setOdom(const nav_msgs::OdometryConstPtr &odom);
    PID angleCtrl_, velCtrl_;               ///< PID controller for angle and velocity


protected:


    double x_, y_, v_, w_, theta_, dtheta_; ///< position, linear velocity, current angle and angle increment
    bool odom_recv_once_;                   ///< check if odometry was at least received once
};

/**
 * @brief The Turn class rotates the turtlebot to an specified angle
 */
class Turn : public Move{

    friend void PublishCmdVel(double angular, double speed, const ros::Publisher &vel_cmd); ///< friend function that publishes velocity commands

public:

    Turn();

    /**
     * @brief move: rotate the robot
     * @param dt: thread execution time ~> important for PID controler
     * @param vel_cmd: Reference to ros::Publisher
     */
    void move(double dt, const ros::Publisher &vel_cmd);

    /**
     * @brief reachedGoal
     * @return return true if goal is reached, false otherwise. Goal is reached if rotation is almost equal to specified angle.
     */
    bool reachedGoal();

    /**
     * @brief turnBy: Rotate turtlebot by specific angle
     * @param angle: angle in rad
     * @param thresh: precision parameter. 0.1 rad by default
     * @return true if angle could be set, false otherwise
     */
    bool turnBy(double angle, double thresh = .1);

private:

    double target_angle_;  ///< target angle
    double current_angle_; ///< current angle
    double thresh_;        ///< threshold ~> precision parameter

};

/**
 * @brief The MoveByDistAngle class moves the turtlebot to a target that lies in a distant and has an angular offset to turtlebots pose
 */
class MoveByDistAngle : public Move{

    friend void PublishCmdVel(double angular, double speed, const ros::Publisher &vel_cmd);

public:

    MoveByDistAngle();

    /**
     * @brief move: guide turtlebot to target
     * @param dt: thread execution time ~> important for PID controler
     * @param vel_cmd: Reference to ros::Publisher
     */
    void move(double dt, const ros::Publisher &vel_cmd);

    /**
     * @brief reachedGoal
     * @return return true if goal is reached, false otherwise. Goal is reached if distance from robot to goal is smaller than specified threshold
     */
    bool reachedGoal();

    /**
     * @brief moveByDistAngle: Move the robot by dist meters, with angular target offset "angle"
     * @param dist: distance to goal
     * @param angle: angular offset to goal
     * @param thresh: precision paremeter. 0.2 meters by default
     * @return true if parameters could be set, false otherwise
     */
    bool moveByDistAngle(double dist, double angle, double thresh = .2);

private:

    double thresh_;       ///< threshold ~> precision parameter
    Vector2D pos_, goal_; ///< relative position and goal
    Vector2D diff_;       ///< vector from current pos to goal

};

class MoveToPos : public Move{

public:

    MoveToPos();

    void move(double dt, const ros::Publisher &vel_cmd);
    bool reachedGoal();

    bool moveFromPosToGoal(const pose_t & start, const pose_t &goal, double thresh =.2);

private:

    double thresh_;
    pose_t pos_, goal_;
    Vector2D diff_;


};

class MoveWithOdom : public Move{

public:

    MoveWithOdom();

    void move(double dt, const ros::Publisher &vel_cmd);
    bool reachedGoal();
    bool moveToOdomPos(const Vector2D &goal, double thresh = .2);

private:

    double thresh_;
    Vector2D goal_;
    Vector2D diff_;

};



#endif /* MOVE_H */



