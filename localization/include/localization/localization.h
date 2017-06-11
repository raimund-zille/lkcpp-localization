#ifndef LOCALIZATION
#define LOCALIZATION
//#include "util.h"
#include <vector>

#define STDDEV_R 0.1
#define STDDEV_PHI 0.1
#define STDDEV_IDF 0.1
#if 0
typedef struct {   /// pose of robot (coordinate and angular direction)
    int x;         /// x coordinate
    int y;         /// y coordinate
    double theta;  /// angular direction
} pose_t;


typedef struct {   /// turtlebot specific noise parameters for odometry
    float alpha_1;
    float alpha_2;
    float alpha_3;
    float alpha_4;
} turtlebot_odom_params;

typedef struct {
    double r;      /// distance to feature
    double phi;    /// angular to feature
    double specifier;   ///specifier to feature
    int knownCorr; /// Index of landmark if clearly (blue, yellow squares) -1 if unknown(green)
} feature;

typedef struct {
    double x;      /// x coordinate
    double y;      /// y coordinate
    double specifier;   /// specifier
} landmark;

/**
 * @brief Sample new robot pose with gathered odometry data
 * @param odom_pose, measured odometry pose in the robots local coordinate frame
 * @param prev_odom_pose, previous measured odometry pose in therms of the robots local coordinate frame
 * @param real_pose, a real pose in therms of the map coordinate frame
 * @param p, turtlebot specific odometry parameters (noise)
 */
pose_t sample_motion_model_odometry(const pose_t &odom_pose, const pose_t &prev_odom_pose, const pose_t &real_pose, const turtlebot_odom_params &p);

/**
 * @brief augmentedMCL Monte Carlo Localization
 * @param particle_old old Particles
 * @param particle_new new Particles after movement and measurement
 * @param odom_new odometry params
 * @param odom_old odometry params
 * @param features found features
 * @param landmarks landmarks in map
 */
void augmentedMCL(const std::vector<particle> &particle_old, std::vector<particle> &particle_new, const pose_t &odom_new, const pose_t &odom_old, const std::vector<feature> &features, const std::vector<landmark> &landmarks );

/**
 * @brief landmarkModelKnownCorr calculate weight of particle with given feature if you have knowledge about the corresponding landmark
 * @param feature given feature
 * @param pos current position
 * @param corrLandmark landmark corresponding to the feature
 * @return weight
 */
double landmarkModelKnownCorr(const feature &feature, const pose_t &pos, const landmark &corrLandmark);

/**
 * @brief landmarkModelUnknownCorr calculate weight of particle with given feature if you dont have knowledge about the corresponding landmark
 * @param feature given feature
 * @param pos current position
 * @param landmarks all known landmarks
 * @return maximum weight of all combinations
 */
double landmarkModelUnknownCorr(const feature &feature, const pose_t &pos, const std::vector<landmark> &landmarks);
#endif
#endif
