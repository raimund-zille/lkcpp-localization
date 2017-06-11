#ifndef CRUISECONTROL_H
#define CRUISECONTROL_H

/* ROS header files */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

/* C++ header files */
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <string>

/* costum header files */
#include "move.h"
#include "particle.h"

/* default thread execution time in (s) ~> 100 Hz*/
#define DT (1./10.)

/**
 * @brief The CruiseControl class controls the movement of the Turtlebot.
 * Make sure that ros::spin() or ros::spinOnce() is executed in your main thread, else you won't get any movement =)
 * Also make sure that thread executes in (ms) time frame
 */
class CruiseControl{

public:

    CruiseControl(ros::NodeHandle &nh, const std::string &odom_sub = "/odom", const std::string &twist_topic = "/cmd_vel_mux/input/teleop", double dt = DT);
    ~CruiseControl();

    /**
     * @brief moveTo: move an angle and a specific distance
     * @param angle_diff: angle difference (angle_to_goal - current_robot_angle)
     * @param dist: distance to move
     * @param thresh: precision parameter, default 20cm
     * @return true if parameters could be set, false otherwise
     */
    bool moveTo(double dist, double angle_diff, double thresh = .2);

    /**
     * @brief moveTo: move from start to goal
     * @param start
     * @param goal
     * @param thresh: precision parameter, default 20 cm
     * @return true if parameters could be set, false otherwise
     */
    bool moveTo(const pose_t &start, const pose_t &goal, double thresh = 0.2);

    /**
     * @brief moveToOdomPos: move to a goal defined in the odometry frame
     * @param goal: goal in odom frame
     * @param thresh: precision parameter, default 20 cm
     * @return true if parameters could be set, false otherwise
     */
    bool moveToOdomPos(const Vector2D &goal, double thresh = .2);


    /**
     * @brief turn: rotate the robot
     * @param angle: angle in rad to rotate
     * @param thresh: precision threshold
     * @return true if parameters could be set, false otherwise
     */
    bool turn(double angle, double thresh);

    /**
     * @brief stop: stop thread ~> no velocity values are published
     */
    void stop();

    /**
     * @brief start: start thread
     * @param dt: optional thread execution time
     */
    void start(double dt = DT);

    /**
     * @brief reachedGoal: check if goal was reached
     * @return true if goal is reached, false otherwise. Goal can be of type "Turn" or "MoveByAngleDist". Ref. "move.h" for details
     */
    bool reachedGoal();


private:



    ros::NodeHandle nh_;        ///< ros nodehandle
    ros::Subscriber odom_sub_;  ///< odometry subscriber
    ros::Publisher twist_pub_;  ///< twist message publisher

    boost::thread *thr;         ///< boost thread
    boost::mutex guard_;        ///< mutex to guarantee thread safe data access

    Turn *turn_;                ///< pointer to Turn object
    MoveByDistAngle *mbda_;     ///< pointer to MoveByDistAngle object
    MoveToPos *mtp_;            ///< pointer to MoveToPos object
    MoveWithOdom *mwo_;         ///< pointer to MoveWithOdom object

    Move *mv_;                  ///< pointer to abstract base class. Function can be changed during runtime.

    double dt_;                 ///< thread execution time

    bool odom_received_;
    bool execute_thread_;
    bool run_thread_;           ///< run the thread if true else not

    void wait_for_odom();

    /**
     * @brief run thread. Executes "mv_->move()"
     */
    void run();

    /**
     * @brief odom_cb: Odometry callback function
     * @param odom: Pointer to odometry data
     */
    void odom_cb(const nav_msgs::OdometryConstPtr &odom);

};
#endif /* CRUISECONTROL_H */

