#ifndef IMAGE_PROCESSING
#define IMAGE_PROCESSING

/* AUSUS FOV PARAMS in DEGREE*/
#define ASUS_FOV_H 58.0
#define ASUS_FOV_V 45.0

//ROS headers
#include "ros/ros.h"
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_geometry/pinhole_camera_model.h>

// Open CV
#include <opencv2/core/core.hpp>
#include "blob.h"
#include "ImageProcessingObjects.h"
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <thread>
#include "Helper/LineSegment.h"

#define DEBUG_SHOW_IMAGE false
#define cv_imshowDebug(window, image); if (DEBUG_SHOW_IMAGE){ cv::imshow(window, image); }

class Lidar;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> SyncPolicy;


class ImageProcessing{

public:
    /// Constructor
    ImageProcessing(ros::NodeHandle &nh, Lidar *lidar);
    ~ImageProcessing();

    /**
     * @brief calculateBlobs: Calculates blobs from image_raw_ (using depth_raw_)
     * to calculate all blobs and saves them to objects_.
     */
    void calculateBlobs();
    /**
     * @brief showProcessedImage: Helper function to get image_raw_ with blobs in it
     * @return image_raw_ with blobs in it
     */
    cv::Mat showProcessedImage();
    /**
     * @brief getObjects: return found ImageProcessingObjects and sets objects_used_ to true
     * @return objects
     */
    ImageProcessingObjects getObjects() { objects_used_ = true; return objects; }
    /**
     * @brief getObjectsPointer: return pointer to found ImageProcessingObjects and sets objects_used_ to true
     * @return pointer to objects
     */
    ImageProcessingObjects* getObjectsPointer() { objects_used_ = true; return &objects; }

    /**
     * @brief distanceToBlobs: Calc distance to blobs with lidar
     * @param blobs vector of blobs
     * @param lidar Lidar object
     * @retur new blobs with distance set
     */
    std::vector<Blob> distanceToBlobs(std::vector<Blob> blobs, Lidar &lidar);
    /**
     * @brief distanceToBlobs: Calc distance to blobs with kinect
     * @param blobs Vector of blobs
     * @return new blobs with distance set
     */
    std::vector<Blob> distanceToBlobs(std::vector<Blob> blobs);

    /**
     * @brief drawBlobsIn: Draw vector of "blobs" in "image"
     * @param image: Call by reference, image to draw blobs in
     * @param blobs to draw in image
     */
    void drawBlobsIn(cv::Mat &image, std::vector<Blob> blobs);

    /**
    * @brief isPuckAtRobot: Robot has Puck
    * @return true, if puck of teamcolor is caught
    */
    double isPuckAtRobot();

    /**
     * @brief objects_ok: Check if objects were calculated
     * @return true, if objects were calculated, else false
     */
    bool objects_ok(void) { return objects_ok_; }
    /**
     * @brief objects_used: Check if objects were used
     * @return true, if objects were used
     */
    bool objects_used(void) { return objects_used_; }
    /**
     * @brief image_ok: check if image is ok
     * @return true, if image is usable
     */
    bool image_ok(void);
    /**
     * @brief depth_ok: check if depth image is ok
     * @return true, if depth image is usable
     */
    bool depth_ok(void);

    /**
     * @brief getDistBetweenTwoBlobs: Calculate the distance between two blobs in the world frame
     * @param blob1
     * @param blob2
     * @return distance between blob1 and blob2 in meters
     */
    double getDistBetweenTwoBlobs(Blob blob1, Blob blob2);
    /**
     * @brief drawBlobsInMap: Fills map with blobs
     * @param image --> call by reference, but empty mat is possible as well
     * @param blobs to draw
     * @param scale to scale blobs in image
     */
    void drawBlobsInMap(cv::Mat& image, const std::vector<Blob>& blobs, double scale = 100);
    /**
     * @brief drawPointsInMap: Draw point in map
     * @param image --> call by reference, but empty mat is possible as well
     * @param points to draw
     * @param color for points in map
     */
    void drawPointsInMap(cv::Mat& image, const std::vector<cv::Point2f>& points, cv::Scalar color);
    /**
     * @brief drawPointInMap: Draw points in map
     * @param image --> call by reference, but empty mat is possible as well
     * @param points to draw
     * @param color for points in map
     */
    void drawPointInMap(cv::Mat& image, const cv::Point2f &p, cv::Scalar color);

    /**
     * @brief drawPositionInMap: Draw pose in map
     * @param image --> call by reference, but empty mat is possible as well
     * @param p pose to draw
     * @param color to draw pose in map
     */
    void drawPositionInMap(cv::Mat& image,const geometry_msgs::Pose &p,cv::Scalar color);
    /**
     * @brief drawLine in image
     * @param image --> call by reference, but empty mat is possible as well
     * @param p line
     * @param color to draw line in
     */
    void drawLine(cv::Mat& image, const Line &p, cv::Scalar color);
    /**
     * @brief drawLineSegment in image
     * @param image --> call by reference, but empty mat is possible as well
     * @param p line
     * @param color to draw line in
     */
    void drawLineSegment(cv::Mat& image, const LineSegment &p, cv::Scalar color);
    /**
     * @brief drawVector2d in image
     * @param image --> call by reference, but empty mat is possible as well
     * @param p vector
     * @param color to draw vector in
     */
    void drawVector2d(cv::Mat& image, const Vector2d &p, cv::Scalar color);
    /**
     * @brief calcKmeans: draw blobs of kmeans and centers in image
     * @param image to draw in
     * @param blobs
     * @return
     */
    double calcKmeans(cv::Mat& image, std::vector<Blob> &blobs);
    /**
     * @brief calcKmeans:  perform Kmeans on vector of blobs
     * @param blobs
     * @param centers return value by reference
     * @return
     */
    double calcKmeans(std::vector<Blob> &blobs, std::vector<Vector2d> &centers);

    /// Local saved images.
    cv::Mat image_raw_, image_processed_,  image_gray_raw_, depth_raw_;
    cv::Mat image_bottom_, image_to_draw_;
    bool isYellowTeam_;

private:
    /// Detected Objects.
    ImageProcessingObjects objects;

    /// Callback Functions
    void syncCallback(const sensor_msgs::CompressedImageConstPtr& image_msg,
                      const sensor_msgs::ImageConstPtr& depth_msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    /// Calc distance with kinect
    /**
     * @brief getDistanceFromKinect using one point
     * @return
     */
    double getDistanceFromKinect(cv::Point2i);
    /**
     * @brief getMeanDistanceFromKinect for a Blob
     * @param blob
     * @return
     */
    double getMeanDistanceFromKinect(Blob blob);
    /**
     * @brief getMedianDistanceFromKinect for a blob
     * @param blob
     * @return
     */
    double getMedianDistanceFromKinect(Blob blob);
    /**
     * @brief draw distance from lidar in image
     * @return image
     */
    cv::Mat drawLidar();

    /// Object detection function
    /**
     * @brief threadedFindBlobsInImage helper function to threadedly find blobs of different colors
     * @param color of blobs to find
     */
    void threadedFindBlobsInImage(ImageProcessingTypes::COLOR_TYPES color);
    /**
     * @brief findBlobsInImage
     * @param image to find blobs in
     * @param color color of blobs to find
     * @return
     */
    std::vector<Blob> findBlobsInImage(cv::Mat &image, ImageProcessingTypes::COLOR_TYPES color);
    /**
     * @brief filterSimilarBlobs using different, empirical determined hyper parameters
     * @param blobs to filter
     * @param angleDegree
     * @param distanceDiff
     * @return
     */
    std::vector<Blob> filterSimilarBlobs(std::vector<Blob> &blobs, float angleDegree, float distanceDiff);
    /**
     * @brief filterBottom from image: delete blobs from image if bottom
     * @param blobs
     * @return
     */
    std::vector<Blob> filterBottom(std::vector<Blob> &blobs);
    void filterPucks(std::vector<Blob> &blobs);
    void filterRobot(std::vector<Blob> &blobs);
    void filterGreenPosts(std::vector<Blob> &blobs);
    void filterPucksWithGreenPosts(std::vector<Blob> &pucks, std::vector<Blob> &greenPosts);
    void shrinkContourOfBlobs(std::vector<Blob> &blobs);
    /// Use rgb image from calllback and filter for specified color and return the filtered image.
    /// \param image : image where blobs get highlighted
    /// \param color : filter for this specific color
    cv::Mat extractBlobsInImage(cv::Mat &image, ImageProcessingTypes::COLOR_TYPES color);

    /// Image manipulation functions:
    void cutPointsWithKinect(Blob &blob, double difference = 0.3);
    void cutDepthImageFromImage(cv::Mat &image, double maxDistance);
    void cutDistanceAboveImageFromImage(cv::Mat &image, double distanceToFilter);
    cv::Mat cutBlobsFromImage(std::vector<Blob> blobs);

    /// \brief Draw marker, distance and angle of a blob into the passed image.
    /// \param image:  Draw in this image.
    /// \param blob:   This blob is used to draw.
    void drawBlobInfoIn(cv::Mat &image, Blob blob);

    /// \brief Get the horizontal angle from pixel x coordinate to image center.
    double getHorizontalAngle(double x_coordinate);

    /// ROS Subscriber and Synchronizer
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> *image_subscriber_;
    message_filters::Subscriber<sensor_msgs::Image> *depth_subscriber_;
    ros::Subscriber poseSub_;
    nav_msgs::Odometry odom_;
    tf::TransformListener listener;
    message_filters::Synchronizer<SyncPolicy>* sync_;

    /// Local lidar
    Lidar *lidar_;

    /// Indicators, if an image is received correctly.
    int counter = 0;
    bool objects_ok_ = false;
    bool objects_used_ = false;
    bool images_new_ = false;
    bool img_ok_;
    bool depth_ok_;
    bool scan_ok_;


    /// DEBUG and optimizing Stuff:
    int iLowH = 0;
    int iHighH = 179;
    int iLowS = 0;
    int iHighS = 255;
    int iLowV = 0;
    int iHighV = 255;
    int epsilon = 1;
    int th1 = 1;
    int th2 = 5;
    std::thread t1,t2,t3,t4,t5,t6;
};
#endif
