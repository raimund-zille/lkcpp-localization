#ifndef DRIVE_H
#define DRIVE_H

/* ros header files */
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

/* c++ header files */
#include <string>
#include <vector>
#include <list>
#include <boost/thread.hpp>

/* costum libraries */
#include "particle.h"
#include "purepursuit.h"
#include "vector2d.h"
#include "PID.h"

#define RATE 20.
#define FORWARD_SPEED_MPS .5               // meter per second
#define ANGLE_MIN_RPS     (-20./180.*M_PI) // rad per second
#define ANGLE_MAX_RPS     (20./180.*M_PI)  // rad per second

/* start values for angle pid controller */
#define P_START           .1
#define I_START           .0
#define D_START           .0

enum status_t{
  IDLE,
  FETCH_WAYPOINT,
  DRIVING
};

/**
 * @brief The Drive class is in charge for moving the robot with as close as possible
 * to a given route
 */
class Drive{

public:

    Drive(ros::NodeHandle &nh, const std::string &pose = "amcl_pose", const std::string &goals = "navigation/mypath");

    /**
     * @brief drive: move the turtlebot with the knowledge of route
     *
     */
    void drive();


private:

    ros::Subscriber nav_goals_sub_;                ///< nav goals subscriber
    ros::Subscriber pose_sub_;                     ///< pose subscriber
    ros::Publisher  twist_pub_;                    ///< twist publisher
    ros::Publisher  marker_pub_;                   ///< marker array publisher
    ros::NodeHandle nh_;                           ///< node handle
    tf::TransformListener listener_;

    std::list<geometry_msgs::PoseStamped> pose_;   ///< list of nav goal poses
    geometry_msgs::PoseWithCovarianceStamped  amcl_pose_;

    boost::thread *thread;                         ///< thread variable for keyboard
    boost::mutex mtx_;

    bool run_thread_;                              ///< thread runs if true, terminates otherwise

    void nav_goals_callback(const nav_msgs::Path::ConstPtr &nav_msg);
    void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose);

    /**
     * @brief keyboard_thread reads user input of keyboard
     */
    void keyboard_thread();

    void PublishCmdVel(double angular, double speed);

    void PublishMarker(const std::vector<Vector2D> &path_approx);

    bool convertToRobotFrame(const std::vector<Vector2D> &path, std::list<Vector2D> &vout);

    bool convertToRobotFrame(const geometry_msgs::PoseWithCovarianceStamped &p, pose_t &out);
};

#endif /* DRIVE_H */
