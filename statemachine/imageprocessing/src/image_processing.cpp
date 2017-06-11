#include "image_processing.h"
#include "util.h"
#include "lidar_processing.h"
#include "opencv2/opencv.hpp"
#include <boost/math/special_functions/sign.hpp>
#include <angles/angles.h>


using namespace std;
using namespace cv;

Point2f centerOfRect(Rect rect) {
    float centerx = rect.tl().x + rect.width/2;
    float centery = rect.tl().y + rect.height/2;
    return Point2f(centerx,centery);
}

ImageProcessing::ImageProcessing(ros::NodeHandle &nh, Lidar *lidar): nh_(nh), img_ok_(false), depth_ok_(false){
    lidar_ = lidar;
    image_transport::TransportHints hints("compressed", ros::TransportHints());
    image_subscriber_ = new message_filters::Subscriber<sensor_msgs::CompressedImage>(nh_, "/camera/rgb/image_raw/compressed", 1); //
    depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/camera/depth_registered/image_raw", 1);
    poseSub_ = nh_.subscribe("/odom", 40, &ImageProcessing::odomCallback, this);

    sync_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(20), *image_subscriber_, *depth_subscriber_);
    sync_->registerCallback(boost::bind(&ImageProcessing::syncCallback, this, _1, _2)); //, _3

    ROS_INFO ("ImageProcessing is initialized");

//    cv::namedWindow("Control", CV_WINDOW_AUTOSIZE);
//    //Create trackbars in "Control" window
//    cvCreateTrackbar("Distance", "Control", &iLowH, 179); //Hue (0 - 179)
//    cvCreateTrackbar("Distance High", "Control", &iHighH, 179);

//    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
//    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

//    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
//    cvCreateTrackbar("HighV", "Control", &iHighV, 255);
}

ImageProcessing::~ImageProcessing(){
    nh_.shutdown();
    delete image_subscriber_;
    delete depth_subscriber_;
    delete sync_;
}

void ImageProcessing::calculateBlobs()
{
    if (image_raw_.empty()) { return; }
    if (!img_ok_ || !depth_ok_ || !images_new_)
    {
        //ROS_INFO_STREAM("calculateBlobs: Image or depth not available yet --> return");
        objects_ok_ = false;
        return;
    }

   // ROS_INFO("Extract objects");
    ros::Time timeStart = ros::Time::now();
//    Mat img;
    image_raw_.copyTo(image_bottom_);
    image_raw_.copyTo(image_processed_);
    // Filter far objects.
    cutDepthImageFromImage(image_processed_, 4.);
    cutDistanceAboveImageFromImage(image_bottom_, 0.8);

    ros::Duration timePreprocessed = ros::Time::now() - timeStart;
    counter += 3;

    t1 = thread(&ImageProcessing::threadedFindBlobsInImage, this, ImageProcessingTypes::YELLOW);
    t2 = thread(&ImageProcessing::threadedFindBlobsInImage, this, ImageProcessingTypes::GREEN);
    t3 = thread(&ImageProcessing::threadedFindBlobsInImage, this, ImageProcessingTypes::BLUE);
    t4 = thread(&ImageProcessing::threadedFindBlobsInImage, this, ImageProcessingTypes::BLACK);
    t5 = thread(&ImageProcessing::threadedFindBlobsInImage, this, ImageProcessingTypes::BOTTOM_BLUE);
    t6 = thread(&ImageProcessing::threadedFindBlobsInImage, this, ImageProcessingTypes::BOTTOM_YELLOW);
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
    t6.join();
    ros::Duration timeExtracted = ros::Time::now() - timeStart;
    objects.isYellowTeam_ = isYellowTeam_;
   // ROS_INFO_STREAM("Preprocess : " << timePreprocessed << "  extracted: " << timeExtracted);

//    cv_imshowDebug("Blobs extracted ", image_processed_);
//    cv_imshowDebug("Bottom extracted", img);
    objects_ok_ = true;
    objects_used_ = false;
    images_new_ = false;
}

void ImageProcessing::threadedFindBlobsInImage(ImageProcessingTypes::COLOR_TYPES color) {
    switch(color) {
    case ImageProcessingTypes::BOTTOM_BLUE: {
        objects.bottomBlue = findBlobsInImage(image_bottom_, color);
        objects.bottomBlue = filterBottom(objects.bottomBlue);
        break; }
    case ImageProcessingTypes::BOTTOM_YELLOW: {
        objects.bottomYellow = findBlobsInImage(image_bottom_, color);
        objects.bottomYellow = filterBottom(objects.bottomYellow);
        break; }
    case ImageProcessingTypes::YELLOW:
        objects.yellowPucks = findBlobsInImage(image_processed_, color);
        filterPucks(objects.yellowPucks);
        objects.yellowPucks = filterSimilarBlobs(objects.yellowPucks, 3, 0.25);
        break;
    case ImageProcessingTypes::BLUE:
        objects.bluePucks = findBlobsInImage(image_processed_, color);
        filterPucks(objects.bluePucks);
        objects.bluePucks = filterSimilarBlobs(objects.bluePucks, 3, 0.25);
        break;
    case ImageProcessingTypes::BLACK:
        objects.blackRobot = findBlobsInImage(image_processed_, color);
        filterRobot(objects.blackRobot);
        objects.blackRobot = filterSimilarBlobs(objects.blackRobot, 6, 0.3);
        break;
    case ImageProcessingTypes::GREEN:
        objects.greenPosts = findBlobsInImage(image_processed_, color);
        filterGreenPosts(objects.greenPosts);
        objects.greenPosts = filterSimilarBlobs(objects.greenPosts, 1., 0.2);
        break;
    default : break;
    }
}

cv::Mat ImageProcessing::showProcessedImage() {
    if (image_raw_.empty()) { return Mat(); }
    ros::Time timeStart = ros::Time::now();
    Mat imageToDisplay;
    image_raw_.copyTo(imageToDisplay);
    drawBlobsIn(imageToDisplay, objects.bluePucks);
    drawBlobsIn(imageToDisplay, objects.greenPosts);
    drawBlobsIn(imageToDisplay, objects.yellowPucks);
    drawBlobsIn(imageToDisplay, objects.bottomBlue);
    line(imageToDisplay,Point(1,0.7 * imageToDisplay.size().height), Point(imageToDisplay.size().width - 1,0.7 * imageToDisplay.size().height),
         Scalar(0,0,0),4);
    ros::Duration timeImPlotted = ros::Time::now() - timeStart;
    //  ROS_INFO_STREAM("ImagePlotting time : " << timeImPlotted);

    return imageToDisplay;
}

Mat detect;
double ImageProcessing::isPuckAtRobot() {
    vector<Point> locations;
    double center = 0;
    if (isYellowTeam_) {
        detect = extractBlobsInImage(image_raw_, ImageProcessingTypes::YELLOW);
        try {
        cv::findNonZero(detect, locations); } catch(const exception&) {}
       // ROS_INFO_STREAM("detect" << locations.size());
        //cv_imshowDebug("detect", detect);
        for(Point p : locations) {
            center += p.x;
        }
        center /= locations.size();
        if (locations.size() > 10000) {
            ROS_INFO_STREAM("DETECT YELLOW PUCK! at " << round(center) << " angle. " << getHorizontalAngle(center));
            return getHorizontalAngle(center);
        }
    } else {
        detect = extractBlobsInImage(image_raw_, ImageProcessingTypes::BLUE);
        try {
        cv::findNonZero(detect, locations); } catch(const exception&) {}
        for(Point p : locations) {
            center += p.x;
        }
        center /= locations.size();
        if (locations.size() > 7000) {
            ROS_INFO_STREAM("DETECT BLUE PUCK! at " << round(center) << " angle. " << getHorizontalAngle(center));ROS_INFO_STREAM("DETECT BLUE PUCK!");
            return getHorizontalAngle(center);
        }
    }
    return 999;
}

void ImageProcessing::syncCallback(const sensor_msgs::CompressedImageConstPtr& image_msg,
                                   const sensor_msgs::ImageConstPtr& depth_msg)
{
    //,  const sensor_msgs::CameraInfoConstPtr& info_msg
    //ROS_INFO_STREAM("Depth and rgb RECEIVED!");

    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        depth_raw_ = cv_ptr->image;
        depth_ok_ = true;

        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        image_raw_ = cv_ptr->image;
        cvtColor(image_raw_, image_gray_raw_, CV_BGR2GRAY);
        img_ok_ = true;
        images_new_ = true;
    }
    catch (cv_bridge::Exception& ex)
    {
        ROS_ERROR("Failed to convert image");
        depth_raw_ = cv::Mat();
        image_raw_ = cv::Mat();
        depth_ok_ = false;
        img_ok_ = false;
    }
}

void ImageProcessing::cutDepthImageFromImage(Mat &image, double maxDistance) {
    //    Mat result;
    //    image_raw_.copyTo(result);
    // ROS_INFO_STREAM(depth_raw_.size() << " and " << image_raw_.size());
    for (int x = 0; x < depth_raw_.size().width; x++) {
        for (int y = 0; y < depth_raw_.size().height; y++) {
            Point2i p(x,y);
            double dist = getDistanceFromKinect(p);
            // ROS_INFO_STREAM(" dist: " << dist);
            if (dist < 0.05 || dist > maxDistance || dist != dist) {
                Vec3b t = image.at<Vec3b>(p);
                t.val[0] = 0;
                t.val[1] = 0;
                t.val[2] = 255;
                image.at<Vec3b>(p) = t;
            }
        }
    }
}

void ImageProcessing::cutDistanceAboveImageFromImage(Mat &image, double distanceToFilter) {
    //    Mat result;
    //    image_raw_.copyTo(result);
    // ROS_INFO_STREAM(depth_raw_.size() << " and " << image_raw_.size());
    for (int x = 0; x < depth_raw_.size().width; x++) {
        for (int y = 0; y < depth_raw_.size().height; y++) {
            Point2i p(x,y);
            double dist = getDistanceFromKinect(p);
            // ROS_INFO_STREAM(" dist: " << dist);
            if (dist > distanceToFilter ) {
                Vec3b t = image.at<Vec3b>(p);
                t.val[0] = 0;
                t.val[1] = 0;
                t.val[2] = 255;
                image.at<Vec3b>(p) = t;
            }
        }
    }
}

Mat ImageProcessing::cutBlobsFromImage(vector<Blob> blobs) {
    Mat result(image_raw_.size().height, image_raw_.size().width, CV_8SC1, Scalar(0));
    for (Blob blob : blobs) {
        blob.drawBlobImageInImage(result, image_gray_raw_);
    }
    return result;
}
//Mat t;

Mat ImageProcessing::extractBlobsInImage(Mat &image, ImageProcessingTypes::COLOR_TYPES color)
{
    cv::Mat color_filtered_image;
    cv::cvtColor(image, color_filtered_image, cv::COLOR_BGR2HSV);
    cv::GaussianBlur(color_filtered_image, color_filtered_image, cv::Size(9, 9), 1, 1);
    //cv::Mat color_filtered_image;

    switch(color){
    case ImageProcessingTypes::BOTTOM_BLUE:{
        //cv::inRange(hsv_image, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), color_filtered_image);
        //cv::inRange(color_filtered_image, Scalar(15, 40, 210), Scalar(40, 255, 255), color_filtered_image); //X 80 X ...
        cv:: inRange(color_filtered_image, Scalar(105, 30, 30), Scalar(130, 255, 255), color_filtered_image);
        break;
    }
    case ImageProcessingTypes::BOTTOM_YELLOW:{
        cv::inRange(color_filtered_image, Scalar(15, 30, 200), Scalar(40, 255, 255), color_filtered_image);
        break;
    }
    case ImageProcessingTypes::YELLOW:{
        //cv::inRange(hsv_image, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), color_filtered_image);
        //cv::inRange(color_filtered_image, Scalar(15, 40, 210), Scalar(40, 255, 255), color_filtered_image); //X 80 X ...
        cv::inRange(color_filtered_image, Scalar(15, 45, 180), Scalar(40, 255, 255), color_filtered_image); //X 80 X ...
        break;
    }
    case ImageProcessingTypes::BLUE:{
        //cv:: inRange(color_filtered_image, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), color_filtered_image);
        cv:: inRange(color_filtered_image, Scalar(105, 30, 30), Scalar(130, 255, 255), color_filtered_image);
        break;
    }
    case ImageProcessingTypes::BLACK:
        cv:: inRange(color_filtered_image, Scalar(0, 0, 0), Scalar(180, 255, 45), color_filtered_image);
        break;
    case ImageProcessingTypes::GREEN:{
        cv:: inRange(color_filtered_image, Scalar(55, 50, 60), Scalar(88, 255, 255), color_filtered_image); //50-90
        break;
    }
    default : break;
    }
    //
    cv::dilate(color_filtered_image, color_filtered_image, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
//cv::dilate(color_filtered_image, color_filtered_image, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
    // Create a structuring element
    int erosion_size = 3;
    cv::Mat element = getStructuringElement(cv::MORPH_RECT ,
                                            cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1));

    // Apply erosion or dilation on the image
    cv::erode(color_filtered_image,color_filtered_image,element);
   // cv::erode(color_filtered_image,color_filtered_image,element);
    return color_filtered_image;
}

/// Returns the blobs in image image
vector<Blob> ImageProcessing::findBlobsInImage(Mat &image, ImageProcessingTypes::COLOR_TYPES color)
{

    /// Find biggest blob
    std::vector<std::vector<cv::Point> > contours;
    vector<Vec4i> hierarchy;
    // Find contours
    cv::Mat contour_image = extractBlobsInImage(image, color);
    cv::findContours( contour_image, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE ); //NONE

    std::vector<cv::Moments> mu(contours.size() );
    const double countourArea_threshold = 400.0;       // to filter the small features and noise

    vector<Blob> blobs;
    for( int i = 0; i < contours.size(); i++ ) {
        mu[i] = cv::moments( contours[i], false );
        double area = contourArea(contours[i]);
        if ((area > countourArea_threshold) && !isContourConvex(contours[i])) {

            Rect rect = boundingRect(contours[i]);

            cv::Point2f tempMassCenter = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
            //Blob blob = Blob(centerOfRect(rect), rect);
            Blob blob = Blob(tempMassCenter, rect);
            blob.contour = contours[i];
            blob.imageSize = cv::Size(image.cols, image.rows);

            double deg = getHorizontalAngle(blob.center.x); // Get Angle to blob
            blob.angleDegree = deg;
            blob.color = color;
            blob.contourArea = area;
            blob.distance = getDistanceFromKinect(Point2i(round(blob.center.x), round(blob.center.y)));
            blob.meanDistance = getMeanDistanceFromKinect(blob);
            double diff = 5;
            if (blob.meanDistance > 3) {
                diff = 3.2;
            }
            blob.lidarDistance = lidar_->getClosestDistance(deg - diff, deg + diff);
            blob.medianDistance =  getMedianDistanceFromKinect(blob);
            tf::StampedTransform transform;
            try{
              listener.lookupTransform("/odom", "/camera_link",
                                       ros::Time(0), transform);
            }
            catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
              //ros::Duration(1.0).sleep();
              continue;
            }
//            geometry_msgs::PointStamped p1;
            geometry_msgs::PoseStamped pos1;
            if (blob.medianDistance != blob.medianDistance || blob.medianDistance > 9) {
                blob.medianDistance = 0.2;
            }
//            p1.header.frame_id = "/camera_link";
//            p1.header.stamp = ros::Time();
            pos1.header.frame_id = "/camera_link";
            pos1.header.stamp = ros::Time();
            pos1.pose.position.y = sin(angles::from_degrees(deg)) * blob.medianDistance;
            pos1.pose.position.x = cos(angles::from_degrees(deg)) * blob.medianDistance;
            pos1.pose.position.z = 0;
            pos1.pose.orientation = tf::createQuaternionMsgFromYaw(angles::from_degrees(deg));
            geometry_msgs::PoseStamped pos2;
            listener.transformPose("/odom",pos1, pos2);
            blob.worldPosition = pos2.pose;

//            p1.point.y = sin(angles::from_degrees(deg)) * blob.medianDistance;
//            p1.point.x = cos(angles::from_degrees(deg)) * blob.medianDistance;
//            p1.point.z = 0;
//            geometry_msgs::PointStamped p2;
//            listener.transformPoint("/odom",p1, p2);
//            blob.pose = odom_.pose.pose;
            //blob.worldPosition.position = p2.point;
            //if (blob.color == ImageProcessingTypes::GREEN)
            //    ROS_INFO_STREAM("x " << p1.point.x << "y"  << p1.point.y);


            blobs.push_back(blob);
        }
    }
    return blobs;
}

vector<Blob> ImageProcessing::filterSimilarBlobs(vector<Blob> &blobs, float angleDegree, float distanceDiff) {
    vector<Blob> newBlobs;
    for (int i = 0; i < blobs.size(); i++) {
        //ROS_INFO_STREAM("BLOB meanDIst: " << blobs[i].meanDistance << "  " << blobs[i].angleDegree);
        if (fabs(blobs[i].meanDistance) < 1E-3 && fabs(blobs[i].angleDegree) < 1E-3) {
            //ROS_INFO_STREAM("CONTINUE BLOB");
            continue;
        }
        for (int j = 0; j < blobs.size(); j++) {
            if (blobs[i] == blobs[j]) {
                //ROS_INFO_STREAM("Same BLOB");
                continue;
            }
            if ((fabs(blobs[i].angleDegree - blobs[j].angleDegree) < angleDegree)
                && fabs(blobs[i].meanDistance - blobs[j].meanDistance) < distanceDiff
                && (blobs[i].color == blobs[j].color)) {
                //ROS_INFO_STREAM("FOUND BLOB" << i << " " << j);
                blobs[i].pointsInsideContour.insert(
                            blobs[i].pointsInsideContour.begin(),
                            blobs[j].pointsInsideContour.begin(),
                            blobs[j].pointsInsideContour.end());
                blobs[i].center = Point2f((blobs[i].center.x + blobs[j].center.x) / 2,
                                         (blobs[i].center.y + blobs[j].center.y) / 2);
                blobs[i].meanDistance = (blobs[i].meanDistance + blobs[j].meanDistance) / 2;
                blobs[i].angleDegree = (blobs[i].angleDegree + blobs[j].angleDegree) / 2;
                blobs[i].contourArea = (blobs[i].contourArea + blobs[j].contourArea) / 2;
                blobs[j].meanDistance = 0;
                blobs[j].angleDegree = 0;
            }
        }
        newBlobs.push_back(blobs[i]);
    }
    return newBlobs;
}

void ImageProcessing::filterGreenPosts(vector<Blob> &blobs) {
    for (vector<Blob>::iterator blob=blobs.begin(); blob!=blobs.end();) {
        blob->shrinkContour(2);
        blob->drawContourInImage(image_processed_);
        cutPointsWithKinect(*blob);

        if (blob->rect.height < 3 * blob->rect.width) {
            blob = blobs.erase(blob);
        } else {
            ++blob;
        }
    }
}
vector<Blob> ImageProcessing::filterBottom(vector<Blob> &blobs) {
    vector<Blob> newBottom;
    Blob newB;
    newB.contourArea = -20;
    if (blobs.size() > 0) {
        if (blobs[0].color == ImageProcessingTypes::BOTTOM_BLUE) {
            for(Blob b : blobs) {
                if ((b.contourArea > newB.contourArea)
                    && (b.center.y > (0.7*image_raw_.size().height))
                    && (b.contourArea > 1000)
                    && (b.rect.height > 0.8 * b.rect.width)) {
                    newB = b;
                }
            }
            if (newB.contourArea > 0) {
                newBottom.push_back(newB);
            }
        } else {
            for(Blob b : blobs) {
                if ((b.contourArea > newB.contourArea)
                    && (b.center.y > (0.7*image_raw_.size().height))
                    && (b.rect.height > 0.9 * b.rect.width)) {
                    newB = b;
                }
            }
            if (newB.contourArea > 0) {
                newBottom.push_back(newB);
            }
        }
    }
    return newBottom;

//    //ROS_INFO_STREAM("Blobs found before filter: " << blobs.size());
//    for (vector<Blob>::iterator blob=blobs.begin(); blob!=blobs.end();) {
//        //blob->shrinkContour(2);
//        //cutPointsWithKinect(*blob);
//       // blob->drawContourInImage(image_processed_);
//        if (blob->rect.height > 0.6 * blob->rect.width
//                && blob->center.y > ((0.5*blob->meanDistance) * image_raw_.size().height)) {
//            ++blob;
//        } else {
//            blob = blobs.erase(blob);
//        }
//    }
}

void ImageProcessing::filterPucks(vector<Blob> &blobs) {
    //ROS_INFO_STREAM("Blobs found before filter: " << blobs.size());
    for (vector<Blob>::iterator blob=blobs.begin(); blob!=blobs.end();) {
        blob->shrinkContour(2);
        cutPointsWithKinect(*blob);
       // blob->drawContourInImage(image_processed_);
        if (blob->rect.height > 1.3 * blob->rect.width
                && blob->contourArea > 700
                && blob->contourArea < 50000
                && blob->meanDistance > 0.05
                && blob->meanDistance < 4
                && blob->center.y > (0.3 * image_raw_.size().height)
                && blob->center.y < (0.7 * image_raw_.size().height)) {
            ++blob;
        } else {
            blob = blobs.erase(blob);
        }
    }
}

void ImageProcessing::filterRobot(vector<Blob> &blobs)
{
    for (vector<Blob>::iterator blob=blobs.begin(); blob!=blobs.end();)
    {
        cutPointsWithKinect(*blob);

        if (/*blob->rect.height < 1.2 * blob->rect.width
                && blob->rect.height > 0.8 * blob->rect.width*/
                blob->center.y > (0.5 * image_raw_.size().height)
                && blob->meanDistance < 2.0
                && blob->meanDistance > 0.2
                && blob->contourArea < 15000
                && blob->contourArea > 500) {
            ++blob;
        } else {
            blob = blobs.erase(blob);
        }
    }
    Blob result;
    result.contourArea = -20;
    for(Blob b : blobs) {
        if (b.contourArea > result.contourArea)
        {
            result = b;
        }
    }
    blobs.clear();
    if (result.contourArea > -20) {
        blobs.push_back(result);
    }
}

void ImageProcessing::shrinkContourOfBlobs(vector<Blob> &blobs) {
    for (Blob &blob : blobs) {
        blob.shrinkContour(1);
    }
}

void ImageProcessing::filterPucksWithGreenPosts(vector<Blob> &pucks, vector<Blob> &greenPosts) {
    //    for (vector<Blob>::iterator blob=blobs.begin(); blob!=blobs.end();) {

    //        if (blob->rect.height < 4 * blob->rect.width) {
    //            blob = blobs.erase(blob);
    //        } else {
    //            ++blob;
    //        }
    //    }
}

/// Returns blob vector with distances and center of each blob
/// Param: centers of the blobs
vector<Blob> ImageProcessing::distanceToBlobs(vector<Blob> blobs, Lidar &lidar) {
    for (Blob blob : blobs) {
        double deg = blob.angleDegree;
        blob.distance = lidar.getClosestDistance(deg-3., deg+3.);
    }
    return blobs;
}

/// Returns blob vector with distances and center of each blob
vector<Blob> ImageProcessing::distanceToBlobs(vector<Blob> blobs)
{
    /* get angle and distance to blob */
    for (Blob &blob : blobs) {
        cv::Point2i point = Point2i(round(blob.center.x), round(blob.center.y));
        //blob.angleDegree = getHorizontalAngle(blob.center.x); // Get Angle to blob
        blob.meanDistance =  getMeanDistanceFromKinect(blob);
        blob.medianDistance =  getMedianDistanceFromKinect(blob);
        blob.distance = getDistanceFromKinect(point);
        //ROS_INFO_STREAM(blob.distance);
    }
    return blobs;
}

void ImageProcessing::drawBlobsIn(Mat &image, std::vector<Blob> blobs) {
    for (Blob blob : blobs) {
        drawBlobInfoIn(image, blob);
        //rectangle(image, blob.rect, Scalar(255,255,255),2);
        //blob.drawPointsInsideContourInImage(image);
        //blob.drawContourInImage(image);
        rectangle(image, blob.rect, blob.getScalarColor(), 3);
    }
}

void ImageProcessing::drawBlobInfoIn(Mat &image, Blob blob) {
    cv::Point2f mc = blob.center;
    cv::line(image, cv::Point2f (mc.x-5, mc.y+5), cv::Point2f (mc.x+5, mc.y-5), cv::Scalar(0,0,255),2);
    cv::line(image, cv::Point2f (mc.x+5, mc.y+5), cv::Point2f (mc.x-5, mc.y-5), cv::Scalar(0,0,255),2);
    std::ostringstream ss;
    ss << std::setprecision(3) << blob.lidarDistance;
    string distToBlob = ss.str();
    std::ostringstream ss2;
    ss2 << std::setprecision(3) << -blob.meanDistance << "";
    string angleToBlob = ss2.str();
    //std::string distToBlob = to_string(blob.distance) + "m, \n" + to_string(blob.angleDegree) + "°";
    cv::Point2f pointShow = mc;
    if (pointShow.x > 0.5 * image.size().width) {
        pointShow.x = pointShow.x - 70;
    } else {
        pointShow.x = pointShow.x + 20;
    }
    pointShow.y = pointShow.y + 15;
    cv::putText(image,distToBlob,pointShow,cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,255,0), 1, CV_AA);
    pointShow.y = pointShow.y + 22;
    cv::putText(image,angleToBlob,pointShow,cv::FONT_HERSHEY_COMPLEX_SMALL, 0.9, cv::Scalar(0,255,0), 1, CV_AA);
}

double ImageProcessing::getHorizontalAngle(double x_coordinate){

    /* vertical resolution in Pixel per Degree */
    double resolution_x =  ASUS_FOV_H/image_raw_.size().width;
    int center_of_image_x = image_raw_.size().width / 2;

    /* get the relative offset to center */
    int offset = x_coordinate - center_of_image_x;
    return -offset*resolution_x;

}

/// Returns (kinect) distance from point
/// Param: point to get distance from
double ImageProcessing::getDistanceFromKinect(cv::Point2i point)
{
    double distance =  0; //std::numeric_limits<double>::quiet_NaN();
    if (depth_ok_)
        distance = depth_raw_.at<float>(point);
    return distance;
}

double ImageProcessing::getMeanDistanceFromKinect(Blob blob)
{
    vector<Point> points = blob.getAllPointsInsideContour();
    double dist = 0;
    int count = 0;
    for(Point p : points) {
        double currentDist = getDistanceFromKinect(Point2i(p.x,p.y));
        if (currentDist >= 0.05 && currentDist < 5.5) {
            dist += currentDist;
            count++;
        }
    }
    return points.size() == 0 ? 0 : dist/count;
}

double ImageProcessing::getMedianDistanceFromKinect(Blob blob)
{
    vector<Point> points = blob.getAllPointsInsideContour();
    vector<double> distances;
    double median;
    int size = points.size();

    if (size < 2) {
        return 10;
    }
    for(Point p : points)
    {
        double currentDist = getDistanceFromKinect(Point2i(p.x,p.y));
        if (currentDist >= 0.05 && currentDist < 5.5)
        {
            distances.push_back(currentDist);
        }
    }

    sort(distances.begin(), distances.end());
    int middle = round(distances.size() / 2);
    constraint(middle, 0, (int)distances.size() - 1);
    if (distances.size() <= 0) {
        return 10;
    }
    return distances[middle];
}

void ImageProcessing::cutPointsWithKinect(Blob &blob, double difference)
{
    vector<Point> points = blob.getAllPointsInsideContour();

    // Crazy remove_if using lamda expression, because points.erase was inefficient when called many times
    points.erase(std::remove_if(points.begin(), points.end(),
                                [this,&blob,difference](const Point & p)  // Lambda expression with scopes in []
    {
        double currentDist = getDistanceFromKinect(Point2i(p.x, p.y));
        return currentDist > blob.meanDistance + difference || currentDist < blob.meanDistance - difference;
    }
    ), points.end());

    blob.pointsInsideContour = points;
    if (points.size() > 5) {
        try {
            blob.rect = boundingRect(points);
        } catch(const exception&) {}
    }
}

void ImageProcessing::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_.pose = msg->pose;
}

bool ImageProcessing::image_ok(void)
{
    return img_ok_;
}

bool ImageProcessing::depth_ok(void)
{
    return depth_ok_;
}

double ImageProcessing::getDistBetweenTwoBlobs(Blob blob1, Blob blob2)
{
    int (*sign)(const double&) = &boost::math::sign<double>;

    double distance;
    if (sign(blob1.angleDegree == 0 && sign(blob2.angleDegree == 0))) // Both blobs on one line
        distance = max(blob1.meanDistance, blob2.meanDistance) - min(blob1.meanDistance, blob2.meanDistance);
    else
    {
        // Both angles of Blobs have same sign: alpha = max(of both angles) - min(of both angles)
        // Both angles of Blobs have different sign: alpha = abs(angle1) + abs(angle2)
        double alpha = (sign(blob1.angleDegree) == sign(blob2.angleDegree))
                ? // Angles same sign
                  (max(abs(blob1.angleDegree), abs(blob2.angleDegree)) - min(abs(blob1.angleDegree), abs(blob2.angleDegree)))
                : abs(blob1.angleDegree) + abs(blob2.angleDegree);     // Angles different sign
        alpha = alpha / 180.0 * M_PI;
        distance = sqrt(pow(blob1.meanDistance,2) + pow(blob2.meanDistance, 2) - 2*blob1.meanDistance*blob2.meanDistance*cos(alpha));
    }
    ROS_INFO_STREAM("blob1 angle: " << blob1.angleDegree << " blob1 meandistance: " << blob1.meanDistance << " blob1 medianDistance: " << blob1.medianDistance <<
                    " blob2 angle: "<< blob2.angleDegree << " blob2 meanDistance: " << blob2.meanDistance << " blob2 medianDistance: " << blob2.medianDistance);

    return distance;
}

Mat ImageProcessing::drawLidar() {
    Mat cdst;
    try {
    cv::Mat image =  cv::Mat(1000, 1000, CV_8UC1, cv::Scalar(0));
    int center = 500;
    sensor_msgs::LaserScan  scanPublish = lidar_->scanData;
    double angleIncrementUsed = scanPublish.angle_increment;
    for (int i = 0; i < scanPublish.ranges.size(); i++) {
        double angle = i * angleIncrementUsed;
        double range = scanPublish.ranges[i];
        if ((range > 0 && range < 12) != true) {
            continue; // Skip if range is not valid
        }
        range = range * 50;
        int x = 1, y = 1;
        if (angle > M_PI + M_PI_2) {
            angle = 2 * M_PI - angle;
            x = center - cos(angle) * range;
            y = center + sin(angle) * range;
            image.at<uchar>(y,x) = 255;
        } else if (angle > M_PI) {
            angle = angle - M_PI;
            x = center + cos(angle) * range;
            y = center + sin(angle) * range;
        } else if (angle > M_PI_2) {
            angle = M_PI - angle;
            x = center + cos(angle) * range;
            y = center - sin(angle) * range;
        } else {
            x = center - cos(angle) * range;
            y = center - sin(angle) * range;
        }
        constraint(x,1,999);
        constraint(y,1,999);
        //ROS_INFO_STREAM("x" << x << " y" << y);
        if (scanPublish.intensities[i] > 24)
            image.at<uchar>(y,x) = 255;
    }
    image.at<uchar>(center,center) = 150;
    image.at<uchar>(center - 1,center -1) = 150;
    image.at<uchar>(center+1,center+1) = 150;
    image.at<uchar>(center-1,center+1) = 150;
    image.at<uchar>(center+1,center-1) = 150;

    cvtColor(image, cdst, CV_GRAY2BGR);
    vector<Vec4i> lines;
    HoughLinesP(image, lines, 1, CV_PI/180, 25, 11, 20);
    for( size_t i = 0; i < lines.size(); i++ )
    {
      Vec4i l = lines[i];
      line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
    }
    //cv::imshow("detected lines", cdst);
    //imshow("detected lines", cdst);
    //emit sndFlow(cdst);
    //cv::waitKey(2);
    } catch (int e) {

    }
    return cdst;
}

void ImageProcessing::drawBlobsInMap(cv::Mat& image,const std::vector<Blob>& blobs, double scale)
{
    if (image.empty()) {
       image =  cv::Mat(800, 800, CV_8UC3, cv::Scalar(0,0,0));
    }
    int center = 400;
    for (Blob blob : blobs)
    {
        int x = center + blob.worldPosition.position.x * scale;
        int y = center + blob.worldPosition.position.y * scale;
        if (blob.isLeftSide)
            circle(image, Point(x,y), 1, blob.getScalarColor(), -1, CV_AA);
        else
            circle(image, Point(x,y), 3, blob.getScalarColor(), -1, CV_AA);
    }
}


void ImageProcessing::drawPointsInMap(cv::Mat& image,const std::vector<Point2f>& points,cv::Scalar color)
{
    if (image.empty()) {
       image =  cv::Mat(800, 800, CV_8UC3, cv::Scalar(0,0,0));
    }
    int center = 400;
    for (Point p : points)
    {
        int x = center + p.x * 100;
        int y = center + p.y * 100;
        circle(image, Point(x,y), 3, color, -1, CV_AA);
    }
}

void ImageProcessing::drawPointInMap(cv::Mat& image,const Point2f& p,cv::Scalar color)
{
    if (image.empty()) {
       image =  cv::Mat(800, 800, CV_8UC3, cv::Scalar(0,0,0));
    }
    int center = 400;
    int x = center + p.x * 100;
    int y = center + p.y * 100;
    circle(image, Point(x,y), 2, color, -1, CV_AA);
}

void ImageProcessing::drawPositionInMap(cv::Mat& image,const geometry_msgs::Pose &p,cv::Scalar color)
{
    if (image.empty()) {
       image =  cv::Mat(800, 800, CV_8UC3, cv::Scalar(0,0,0));
    }
    int center = 400;
    int x = center + p.position.x * 100;
    int y = center + p.position.y * 100;
    tf::Quaternion q;
    tf::quaternionMsgToTF(p.orientation, q);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    int x_end = x + 20* cos(yaw);
    int y_end = y + 20* sin(yaw);
    circle(image, Point(x,y), 3, color, -1, CV_AA);
    line(image, Point(x,y), Point(x_end,y_end), color, 1, 8, 0);
}

void ImageProcessing::drawLine(cv::Mat& image, const Line &p, cv::Scalar color) {
    if (image.empty()) {
       image =  cv::Mat(800, 800, CV_8UC3, cv::Scalar(0,0,0));
    }
    int center = 400;
    int x = center + p.getSupportVector().x * 100;
    int y = center + p.getSupportVector().y  * 100;
    int xe= x + p.getDirectionVector().x * 100;
    int ye = y + p.getDirectionVector().y * 100;
    line(image, Point(x,y), Point(xe,ye), color);
}

void ImageProcessing::drawLineSegment(cv::Mat& image, const LineSegment &p, cv::Scalar color) {
    if (image.empty()) {
       image =  cv::Mat(800, 800, CV_8UC3, cv::Scalar(0,0,0));
    }
    int center = 400;
    int x = center + p.getSupportVector().x * 100;
    int y = center + p.getSupportVector().y  * 100;
    int xe= x + p.getEndVector().x * 100;
    int ye = y + p.getEndVector().y * 100;
    line(image, Point(x,y), Point(xe,ye), color);
}


void ImageProcessing::drawVector2d(cv::Mat& image, const Vector2d &p, cv::Scalar color) {
    if (image.empty()) {
       image =  cv::Mat(800, 800, CV_8UC3, cv::Scalar(0,0,0));
    }
    int center = 400;
    int x = center + p.x * 100;
    int y = center + p.y  * 100;
    circle(image, Point(x,y),3, color, -1, CV_AA);
}

double ImageProcessing::calcKmeans(std::vector<Blob> &blobs, vector<Vector2d> &centers) {
    TermCriteria tc;
    centers.clear();
    std::vector<cv::Point2f> points;
    for (Blob &b : blobs) {
        cv::Point2f p;
        p.x =  b.worldPosition.position.x;
        p.y =  b.worldPosition.position.y;
        points.push_back(p);
    }


    if (points.size() >= 4) {
    Mat output, centersTemp;
    try {
    kmeans(points, 2, output,
               tc,
                  3, KMEANS_RANDOM_CENTERS, centersTemp);
    for (int i = 0; i < output.rows; i++) {
        int idx = output.at<int>(i);
        Point2f p = points[i];
        blobs[i].side = idx + 1;
    }
    Mat centers_point = centersTemp.reshape(2,centersTemp.rows);
    if (centersTemp.rows > 1) {
        Point2f clustered_center = centers_point.at<Point2f>(0);
        Point2f clustered_center2 = centers_point.at<Point2f>(1);
        centers.push_back(Vector2d((double)clustered_center.x, (double)clustered_center.y));
        centers.push_back(Vector2d((double)clustered_center2.x, (double)clustered_center2.y));
    }
    } catch(const exception&) {}
    }
}
double ImageProcessing::calcKmeans(cv::Mat& image, std::vector<Blob> &blobs) {
    if (image.empty()) {
       image =  cv::Mat(800, 800, CV_8UC3, cv::Scalar(0,0,0));
    }
    int center = 400;

    TermCriteria tc;
    std::vector<cv::Point2f> points;
    for (Blob &b : blobs) {
        cv::Point2f p;
        p.x =  b.worldPosition.position.x;
        p.y =  b.worldPosition.position.y;
        points.push_back(p);
    }

    RotatedRect rect;
    if (points.size() >= 4) {

        rect = minAreaRect(points);
        Point2f pts[4];
        rect.points(pts);
        for (Point2f p : pts) {
            Point2f draw = p;
            draw.x = center + draw.x * 100;
            draw.y = center + draw.y * 100;
            //ROS_INFO_STREAM(p);
            circle(image,draw, 6, cv::Scalar(50,50,20), -1, CV_AA );
        }
        std::ostringstream ss2;
        ss2 << std::setprecision(3) << rect.size.width / 100 << "m " << rect.size.height / 100  ;
        string t = ss2.str();
        putText(image,t ,Point(50,50),cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,255,0), 1, CV_AA);
    Mat output, centers;
    try {
    kmeans(points, 2, output,
               tc,
                  3, KMEANS_RANDOM_CENTERS, centers);
    for (int i = 0; i < output.rows; i++) {
        int idx = output.at<int>(i);
        Point2f p = points[i];
        blobs[i].side = idx + 1;
        int x =  center + p.x * 100;
        int y =  center + p.y * 100;
        if (idx == 0) {
            //circle(image, Point(x,y), 2, cv::Scalar(0,180,0), -1, CV_AA);
        } else if (idx == 1){
            //circle(image, Point(x,y), 2, cv::Scalar(180,0,0), -1, CV_AA);
        }
    }
    vector<Vec4i> lines;
    Mat im;
//    cv::cvtColor(image, im, CV_BGR2GRAY);
//    HoughLinesP(im, lines, 1, CV_PI/180, 30, 11, 40);
//    for( size_t i = 0; i < lines.size(); i++ )
//    {
//      Vec4i l = lines[i];
//      line( image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
//      double dist = sqrt(pow(l[0] - l[2], 2) + pow(l[1] - l[3], 2)) / 100;
//      std::ostringstream ss2;
//      ss2 << std::setprecision(3) << dist << "m";
//      string t = ss2.str();
//      cv::putText(image,t ,Point(l[0], l[1]),cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,255,0), 1, CV_AA);

//    }
    Mat centers_point = centers.reshape(2,centers.rows);
    if (centers.rows > 1) {
        Point2f clustered_center = centers_point.at<Point2f>(0);
        clustered_center.x = center + clustered_center.x * 100;
        clustered_center.y = center + clustered_center.y * 100;
        circle(image, clustered_center, 5, cv::Scalar(0,255,0), -1, CV_AA);
        Point2f clustered_center2 = centers_point.at<Point2f>(1);
        clustered_center2.x = center + clustered_center2.x * 100;
        clustered_center2.y = center + clustered_center2.y * 100;
        circle(image, clustered_center2, 5, cv::Scalar(255,0,0), -1, CV_AA);
    }
    } catch(const exception&) {}
    }
    return rect.angle;
}
