#include "blob.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


namespace ImageProcessingTypes {
    std::ostream& operator<<(std::ostream& os, const ImageProcessingTypes::COLOR_TYPES& color) {
        switch(color) {
        case ImageProcessingTypes::BLACK:
            os << "Color:   Black" << std::endl;
            break;
        case ImageProcessingTypes::GREEN:
            os << "Color:   Green" << std::endl;
            break;
        case ImageProcessingTypes::YELLOW:
            os << "Color:   Yellow" << std::endl;
            break;
        case ImageProcessingTypes::BLUE:
            os << "Color:   Blue" << std::endl;
            break;
        case ImageProcessingTypes::RED:
            os << "Color:   Red" << std::endl;
            break;
        case ImageProcessingTypes::BOTTOM_BLUE:
            os << "Color:   Bottom Blue" << std::endl;
            break;
        case ImageProcessingTypes::BOTTOM_YELLOW:
            os << "Color:   Bottom Yellow" << std::endl;
            break;
        default:
            os << "Color:   No Color" << std::endl;
        }
        return os;
    }
}

cv::Scalar Blob::getScalarColor() {
    switch(color) {
    case ImageProcessingTypes::BLACK:
        return cv::Scalar(0, 0, 0);
    case ImageProcessingTypes::GREEN:
        return cv::Scalar(0, 255, 0);
    case ImageProcessingTypes::YELLOW:
        return cv::Scalar(0, 150, 150);
    case ImageProcessingTypes::BLUE:
        return cv::Scalar(255, 0, 0);
    case ImageProcessingTypes::RED:
        return cv::Scalar(0, 0, 255);
    case ImageProcessingTypes::BOTTOM_BLUE:
        return cv::Scalar(255, 120, 0);
    case ImageProcessingTypes::BOTTOM_YELLOW:
        return cv::Scalar(0, 120, 255);
    default:
        return cv::Scalar(0, 0, 0);
    }
    return cv::Scalar(0, 0, 0);
}

void Blob::drawContourInImage(Mat &image) {
     vector<vector<Point>> contours;
     contours.push_back(this->contour);
     drawContours(image, contours, 0, getScalarColor(), 2, 8);
}

void Blob::drawPointsInsideContourInImage(cv::Mat &image) {
    //vector<Point> points = getAllPointsInsideContour();
    for (Point point : pointsInsideContour) {
        //ROS_INFO_STREAM(point);
        circle(image, point, 1, getScalarColor());
    }
}

void Blob::drawBlobImageInImage(cv::Mat &image, cv::Mat &originalImage) {
    vector<Point> points = getAllPointsInsideContour();
    for (Point point : points) {
        char t = originalImage.at<char>(point);
        image.at<char>(point) = t;
    }
}

vector<Point> Blob::getAllPointsInsideContour() {
    vector<Point> locations;
    Mat mask = Mat::zeros(imageSize, CV_8UC1);
    vector< vector<Point> > contours;
    contours.push_back(contour);
    drawContours(mask, contours, -1, Scalar(255), CV_FILLED);
    cv::findNonZero(mask, locations);
    return locations;
}

bool compareByProbabiliy(const Blob &a, const Blob &b)
{
    return a.probabilityCounter > b.probabilityCounter;
}
