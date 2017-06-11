#ifndef LIDAR_PROCESSING
#define LIDAR_PROCESSING

//ROS headers
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <mutex>
#include <filter.h>


class Lidar {
public:
    Lidar(ros::NodeHandle &nh);
    sensor_msgs::LaserScan scanData;

    /**
     * @brief getDistanceRaw for one angle
     * @param Deg angle
     * @return
     */
    double getDistanceRaw(int Deg);

    /* Polymorph C++ functions */
    double getClosestDistance(int minAngleDeg, int maxAngleDeg);
//    double getClosestDistance(double minAngleRad, double maxAngleRad, filter_type::Filter *filter = new filter_type::NoFilter());
    double getMeanDistance(int minAngleDeg, int maxAngleDeg);
    //double getMeanDistance(double minAngleRad, double maxAngleRad);

    /**
     * @brief SamplesEmpty check if samples are empty
     * @return
     */
    bool SamplesEmpty(void);
    /**
     * @brief getFilteredData
     * @return
     */
    std::vector<double> getFilteredData(void);

private:
    ros::NodeHandle nh_;
    ros::Subscriber laserScanSubsriber_;
    std::mutex mutex;

    std::vector<std::vector<double>> sensor_ranges;
    std::vector<double> filtered_;

    /**
     * @brief pushSensorData update sensor data (always keep latest 4 measurements)
     * @param data to push
     */
    void pushSensorData(const std::vector<double> &data);
    /**
     * @brief getSensorData callback for ros laserscan topic
     * @param scan
     */
    void getSensorData(const sensor_msgs::LaserScan::ConstPtr& scan);
    /**
     * @brief getLastValue for index
     * @param index
     * @return
     */
    double getLastValue(int index);

    /**
     * @brief validSensorRange check if a sensorRange is valid
     * @param sensorRange
     * @return
     */
    bool validSensorRange(double sensorRange);

};




#endif
