#include "util.h"
#include <iostream>

#define SQRT_2_PI 2.50662827463


double deg2rad(double degree) {
    return degree * M_PI / 180.0;
}

double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}


cv::Point2f castToPoint2f(geometry_msgs::Pose p1) {
    cv::Point2f nP;
    nP.x = p1.position.x;
    nP.y = p1.position.y;
}

geometry_msgs::Pose castToGeometry(cv::Point p1) {
    geometry_msgs::Pose nP;
    nP.position.x = p1.x;
    nP.position.y = p1.y;
}

double getProbGauss(const double &diff, const double &stddev){
    // diff: Wert - mittelwert
    // stddev: Standdardabweichung

    double denom = sqrt(stddev)*SQRT_2_PI;
    double expo = -0.5*pow(diff,2)/stddev;
    double prob = (1./denom)*exp(expo);

    //std::cout <<"probGauss: " << "diff: " << diff << " stddev: " << stddev << " prob: " <<prob << std::endl;
    return prob;
}

double distanceBetweenTwoPoints(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
    return sqrt(pow(p1.position.x - p2.position.x,2) + pow(p1.position.y - p2.position.y,2));
}

double scalar(geometry_msgs::Pose p1,  geometry_msgs::Pose p2) {
    return (p1.position.x * p2.position.x + p1.position.y * p2.position.y);
}

double normPos(geometry_msgs::Pose p1) {
    return sqrt(pow(p1.position.x,2) + pow(p1.position.y,2));
}

//double angleBetweenTwoPoints(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
//    return acos(scalar(p1,p2) / (normPos(p1) * normPos(p2)) );
//}

double angleBetweenTwoPoints(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
    geometry_msgs::Pose res = minus(p1,p2);
    return angleOfPose(res);
}

double angleOfPose(geometry_msgs::Pose &p1) {
    return atan2(p1.position.y,p1.position.x);
}

double angleBetweenTwoPoints(geometry_msgs::Pose &reference, geometry_msgs::Pose &p1, geometry_msgs::Pose &p2) {
    geometry_msgs::Pose p1_ = minus(reference, p1);
    geometry_msgs::Pose p2_ = minus(reference, p2);
    return angleBetweenTwoPoints(p1_,p2_);
}

geometry_msgs::Pose minus(geometry_msgs::Pose &p1, geometry_msgs::Pose &p2) {
    geometry_msgs::Pose  res;
    res.position.x = p1.position.x - p2.position.x;
    res.position.y = p1.position.y - p2.position.y;
    return res;
}


