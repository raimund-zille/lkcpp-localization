#ifndef IMAGE_PROCESSING_OBJECTS_H
#define IMAGE_PROCESSING_OBJECTS_H

#include "blob.h"

class ImageProcessingObjects {
public:
    std::vector<Blob> greenPosts;
    std::vector<Blob> bluePucks;
    std::vector<Blob> yellowPucks;
    std::vector<Blob> blackRobot;

    std::vector<Blob> bottomBlue;
    std::vector<Blob> bottomYellow;

    bool isYellowTeam_;
    /**
     * @brief getOurPucks our team has to drive to goal
     * @return pucks
     */
    std::vector<Blob> getOurPucks() {
        if (isYellowTeam_) {
            return yellowPucks;
        } else {
            return bluePucks;
        }
    }
};

#endif
