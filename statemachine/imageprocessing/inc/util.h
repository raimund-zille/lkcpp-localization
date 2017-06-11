#ifndef UTIL_H
#define UTIL_H
#include <algorithm>
#include <math.h>
#include <vector>
#include "../../localization/amcl/inc/measurement_model.h"
#include <geometry_msgs/Pose.h>
#include "opencv/cv.h"

/* helper functions */
double rad2deg(double degree);
double deg2rad(double rad);
double getProbGauss(const double &diff, const double &stddev);

template<typename T> T constraint(T val, T min, T max) {
    return std::min(std::max(val, min), max);
}

/**
 * @brief castToPoint2f cast pose to point2f
 * @param p1
 * @return
 */
cv::Point2f castToPoint2f(geometry_msgs::Pose p1);
/**
 * @brief castToGeometry cast cv::Point to pose
 * @param p1
 * @return
 */
geometry_msgs::Pose castToGeometry(cv::Point p1);

/**
 * @brief distanceBetweenTwoPoints: calculate distance between two pose points
 * @param p1 point1
 * @param p2 point2
 * @return
 */
double distanceBetweenTwoPoints(geometry_msgs::Pose p1, geometry_msgs::Pose p2);

/**
 * @brief scalar product between two poses
 * @param p1 pose1
 * @param p2 pose2
 * @return
 */
double scalar(geometry_msgs::Pose p1,  geometry_msgs::Pose p2);

/**
 * @brief normPos norm of a pose
 * @param p1 pose
 * @return
 */
double normPos(geometry_msgs::Pose p1);
/**
 * @brief angleOfPose
 * @param p1
 * @return
 */
double angleOfPose(geometry_msgs::Pose &p1);
/**
 * @brief angleBetweenTwoPoints
 * @param reference pose
 * @param p1 point1
 * @param p2 point2
 * @return
 */
double angleBetweenTwoPoints(geometry_msgs::Pose &reference, geometry_msgs::Pose &p1, geometry_msgs::Pose &p2);
/**
 * @brief angleBetweenTwoPoints
 * @param p1 point1
 * @param p2 point2
 * @return
 */
double angleBetweenTwoPoints(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
/**
 * @brief minus pose1 - pose2
 * @param p1
 * @param p2
 * @return
 */
geometry_msgs::Pose minus(geometry_msgs::Pose &p1, geometry_msgs::Pose &p2) ;
#endif
