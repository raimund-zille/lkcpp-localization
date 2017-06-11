#ifndef ROBO_CONTROL_H
#define ROBO_CONTROL_H

//ROS headers
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <blob.h>
#include "Helper/LineSegment.h"
#include "ObjectParser.h"
#include "move.h"


enum class MOVING {
    forward,
    slow,
    very_slow,
    stop
};

enum class TURN {
    left,
    right,
    straight
};

#define CHECK_LASER 0.2
#define THRESHOLD_DIST 0.2

class RoboControl
{
public:
    RoboControl(ObjectParser* objp);
    void move(double angleSpeed, double speed);
    bool move(Blob &b, double speed);
    bool move(Vector2d aim, double pControl, double holdDistance = 0.1, bool useAvoid = false, bool useBackwards = false);
    bool move_by_dist();
    bool turn(Vector2d aim) ;
    void setAngleFromOdom(TURN direction);
    double getTurnedAngle();
    void turn(TURN direction);
    void turnDegree(double degrees);
    void r_phiRad_from_pos(double x_me, double y_me, double phi_me, double x_des, double y_des, double &r, double &phi);
    bool finished_;
    bool driveBackToHome();
    bool driveToGoal();
    geometry_msgs::Pose poseInit_;
    double yawInit_;
    geometry_msgs::Pose currPos_;
    Line currLine_;
    LineSegment lineToGoal_;

    double getMeanDistanceTo(std::vector<Blob> &blobs);
    double getClosestDistanceTo(std::vector<Blob> &blobs) ;
    double getClosestDistanceToUnused(std::vector<Blob> &blobs) ;
    bool canDrive();
    double blocked_r_;
    double blocked_phi_;

    MoveWithOdom move_base_;
    Turn turn_base_;
    MoveByDistAngle move_by_dist_;

    MOVING keepMoving; // Indicates whether the robot should continue moving
private:
    bool checkBlos(BlobArray blobs, geometry_msgs::Pose pos);
    double check_dist(Blob blob, geometry_msgs::Pose pos);
    ros::NodeHandle node;
    ros::Rate rate_;
    ros::Publisher commandPub; // Publisher to the robot's velocity command topic
    ros::Subscriber poseSub;
    ros::Subscriber scanSub;
    double currAngle_;
    geometry_msgs::PoseWithCovariance pose_;
    geometry_msgs::Pose poseAim_;
    ObjectParser* objParser_;
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void laserCallback(const sensor_msgs::LaserScanConstPtr& scan);
    bool wayBlocked_;
    bool checkPhiR(double phi, double r);


    bool odomReceived_;
    double angleSet_;
    TURN direction_;
};
#endif // ROBO_CONTROL_H
