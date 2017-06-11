#ifndef BLOB_H
#define BLOB_H

#include <opencv2/core/core.hpp>
#include <geometry_msgs/Pose.h>

namespace ImageProcessingTypes{
    enum COLOR_TYPES {
        BLACK,
        GREEN,
        YELLOW,
        BLUE,
        RED,
        BOTTOM_YELLOW,
        BOTTOM_BLUE
    };
    std::ostream& operator<<(std::ostream& os, const COLOR_TYPES& color);
}


class Blob {
public:
    Blob() {
        isInitialized = false;
    }
    Blob(cv::Point2f c, cv::Rect r) {
        isInitialized = false;
        center = c;
        rect = r;
    }


    Blob &operator =(const Blob &b) {
        center = b.center;
        rect = b.rect;
        angleDegree = b.angleDegree;
        meanDistance = b.meanDistance;
        medianDistance = b.medianDistance;
        lidarDistance = b.lidarDistance;
        contour = b.contour;
        color = b.color;
        pointsInsideContour = b.pointsInsideContour;
        contourArea = b.contourArea;
        imageSize = b.imageSize;
        pose = b.pose;
        worldPosition = b.worldPosition;
        probabilityCounter = b.probabilityCounter;
        distanceToNextClosestGreenPost = b.distanceToNextClosestGreenPost;
        isInitialized = true;
        transformName = b.transformName;
        isLeftSide = b.isLeftSide;
        alreadyUsedToDrive = b.alreadyUsedToDrive;
        return *this;
    }
    double distance = 10;
    double meanDistance = 10;
    double medianDistance = 10;
    double lidarDistance = 0;
    cv::Point2f center;
    double angleDegree;
    cv::Rect rect;
    std::vector<cv::Point> contour;
    std::vector<cv::Point> pointsInsideContour;
    double contourArea;
    ImageProcessingTypes::COLOR_TYPES color;
    cv::Size imageSize;
    geometry_msgs::Pose pose;
    geometry_msgs::Pose worldPosition;
    int probabilityCounter = 0; /// Delete object when smaller threshold

    /// Only green posts:
    std::string transformName;
    double distanceToNextClosestGreenPost;
    int side = 1; // 1 or 2
    bool isLeftSide = false;

    bool isInitialized = false;
    bool alreadyUsedToDrive = false;

    bool isInGoal = false;

    /**
     * @brief getScalarColor helper function for drawing (provides standard colors)
     * @return
     */
    cv::Scalar getScalarColor();

    /**
     * @brief shrinkContour of blob
     * @param numberOfPoints blob shall be shrinked
     */
    void shrinkContour(int numberOfPoints) {
        for (cv::Point &point : contour) {
            point.x = (point.x < center.x) ? point.x+=numberOfPoints : point.x-=numberOfPoints;
            point.y = (point.y < center.y) ? point.y+=numberOfPoints : point.y-=numberOfPoints;
        }
    }

    /**
     * @brief drawContourInImage: draw blob contour in image
     * @param image to draw blob contour in
     */
    void drawContourInImage(cv::Mat &image);
    /**
     * @brief drawPointsInsideContourInImage
     * @param image to draw points in
     */
    void drawPointsInsideContourInImage(cv::Mat &image);
    /**
     * @brief drawBlobImageInImage
     * @param image to draw in original image
     * @param originalImage to draw in image
     */
    void drawBlobImageInImage(cv::Mat &image, cv::Mat &originalImage);
    std::vector<cv::Point> getAllPointsInsideContour();

    /**
     * @brief operator == overloaded equal comparison
     * @param b blob to compare with this
     * @return
     */
    bool operator == (Blob b) {
          if (std::fabs(meanDistance - b.meanDistance) < 1E-3
                  && std::fabs(angleDegree - b.angleDegree) < 1E-3
                  && (color == b.color)
                  && (center == b.center))
            return true;
          else
            return false;
     }
};
/**
 * @brief compareByProbabiliy using probability counter
 * @param a first blob
 * @param b second blob
 * @return true if same
 */
bool compareByProbabiliy(const Blob &a, const Blob &b);

#endif
