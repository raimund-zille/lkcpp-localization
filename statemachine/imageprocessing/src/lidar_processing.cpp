
#include <lidar_processing.h>
#include <math.h>
#include "util.h"

using namespace std;
using namespace filter_type;

void Lidar::pushSensorData(const vector<double> &data) {
    mutex.lock();
    if (sensor_ranges.size() > 4) {
        sensor_ranges.erase(sensor_ranges.begin());
    }
    sensor_ranges.push_back(data);
    mutex.unlock();
}

Lidar::Lidar(ros::NodeHandle &nh) :nh_(nh){
    laserScanSubsriber_ = nh_.subscribe("/laserscan", 1, &Lidar::getSensorData, this); // changed /laserscan to /Scan
    ROS_INFO ("Lidar is initialized");
}


void Lidar::getSensorData(const sensor_msgs::LaserScan::ConstPtr &scan) {
    scanData.angle_increment = scan->angle_increment;
    scanData.ranges = scan->ranges;
    scanData.intensities = scan->intensities;
    vector<double> scan_data;
    for (int i = 0; i<360; i++) {
        scan_data.push_back(isinf(scan->ranges[i]) ? getLastValue(i) : scan->ranges[i]);

    }
    pushSensorData(scan_data);
    //ROS_INFO_STREAM("MAX RANGE " << scan->range_max);
    //ROS_INFO_STREAM("SENSOR DATA RECEIVED");
}

double Lidar::getLastValue(int index) {
    for (int i = 1; i < sensor_ranges.size(); i++) {
        if ((sensor_ranges[i][index])) {
            return sensor_ranges[i][index];
        }
    }

}

double Lidar::getDistanceRaw(int Deg){
    int angle = Deg % 360;
    int idx = (angle < 0 ? -1 : 1) * angle;
    double val = 0;
    mutex.lock();
    val = sensor_ranges.back()[idx];
    mutex.unlock();
    return val;
}


/// DEGREE
//double Lidar::getClosestDistance(double minAngleDeg, double maxAngleDeg, Filter *filter) {
//    int minIndex = round(minAngleDeg);
//    int maxIndex = round(maxAngleDeg);
//    //ROS_INFO_STREAM("angle  " << minIndex << " " << maxIndex);
//    return getClosestDistance(minIndex, maxIndex, filter);
//}

/// DEGREE
double Lidar::getClosestDistance(int minAngleDeg, int maxAngleDeg) {

    if (sensor_ranges.size() <= 0) {
        return 0;
    }

    int minIndex = minAngleDeg < 0 ? 360 + minAngleDeg : minAngleDeg;
    int maxIndex = maxAngleDeg;
    minIndex = constraint(minIndex, 0, 359);
    maxIndex = constraint(maxIndex, 0, 359);
    if (minIndex > maxIndex) {
        return std::min(getClosestDistance(minIndex, 359), getClosestDistance(0,maxIndex));
    }
    mutex.lock();

      /* clear prevous data and filter if wanted */
    filtered_.clear();
    filtered_ = sensor_ranges.back();
//    shared_ptr<vector<double>> ptr = filter->filter(sensor_ranges);
//    for (auto i = ptr->begin(); i != ptr->end(); i++) filtered_.push_back(*i);

//    for (int i = 0; i < 359; i++) {
//        ROS_INFO_STREAM("filter " << i << "  " << filtered_[i]);
//    }

    double closestRange = 6.0;
    for (int currIndex = minIndex; currIndex <= maxIndex; currIndex++) {
        double currentSensorRange = filtered_[currIndex];
        if (!validSensorRange(currentSensorRange)) continue;
        closestRange = currentSensorRange < closestRange ? currentSensorRange : closestRange;
    }

    /* prevent memory leak */
    //delete filter;

    mutex.unlock();
    closestRange = closestRange  == 6.0 ? 0.15 : closestRange;


    return closestRange;
}

bool Lidar::validSensorRange(double sensorRange) {
    return (sensorRange >= 0.15 && sensorRange <= 6.0);
}

vector<double> Lidar::getFilteredData(){
    return filtered_;
}

double Lidar::getMeanDistance(int minAngleDeg, int maxAngleDeg) {
    if (sensor_ranges.size() <= 0) {
        return 0;
    }
    int minIndex = minAngleDeg < 0 ? 360 + minAngleDeg : minAngleDeg;
    int maxIndex = maxAngleDeg;
    minIndex = constraint(minIndex, 0, 359);
    maxIndex = constraint(maxIndex, 0, 359);
    if (minIndex > maxIndex) {
        return  ((double)(getMeanDistance(minIndex, 359) + getMeanDistance(0,maxIndex)))/2.;
    }
    mutex.lock();
    double meanDistance = 0;
    int samples = 0;

    /* clear prevous data and filter if needed */
//    filtered_.clear();
//    filtered_ = ;
//    shared_ptr<vector<double>> ptr = filter->filter(sensor_ranges);
//    for (auto i = ptr->begin(); i != ptr->end(); i++) filtered_.push_back(*i);

    for (int currIndex = minIndex; currIndex <= maxIndex; currIndex++) {
        double currentSensorRange = sensor_ranges.back()[currIndex];
        if (!validSensorRange(currentSensorRange)) continue;
        samples++;
        meanDistance += currentSensorRange;
    }

    /* prevent memory leak */
    //delete filter;

    mutex.unlock();

    samples = samples == 0 ? 1 : samples;
    return meanDistance / samples;
}

bool Lidar::SamplesEmpty(void){
    return sensor_ranges.empty();
}
