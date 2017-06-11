#ifndef OBJECT_PARSER_H
#define OBJECT_PARSER_H

#include "blobArray.h"
#include "ImageProcessingObjects.h"
#include <vector>
#include "util.h"
#include "tf/transform_broadcaster.h"
#include "StateMachine.h"
#include "states.h"
#include "Helper/Line.h"

class ImageProcessing;
class RoboControl;

class ObjectParser : StateMachine<ObjectParserStates> {
public:
    ObjectParser(ObjectParserStates initState, double frequenzy) : StateMachine<ObjectParserStates>(initState, frequenzy) {}

    ImageProcessing *imageProcessing_;
    RoboControl *rob_;

    Blob closestBlue;
    Blob closestYellow;
    Blob closestPuck;

    Blob biggestBlob;
    tf::TransformBroadcaster br;

    /// Permanent objects
    std::vector<Blob> greenPosts;
    bool greenPostsSplitted = false;
    std::vector<Blob> greenPostsSide1; // SORTET GREEN POSTS - SIDE 1 (4)
    std::vector<Blob> greenPostsSide2; // SORTET GREEN POSTS - SIDE 2 (4)
    std::vector<Blob> greenPostsLeft; // SORTET GREEN POSTS - SIDE 1 (4)
    std::vector<Blob> greenPostsRight; // SORTET GREEN POSTS - SIDE 2 (4)
//    std::vector<Blob> bluePucks;
    BlobArray bluePucks;
    BlobArray yellowPucks;
//    std::vector<Blob> yellowPucks;
    Vector2d puckCenter; // CENTER of all pucks
    Blob leftBackCenter_;
    Line leftLine; // LEFT line starting from leftBackCenter Puck, directing to enemy
    bool leftLineIsSet = false;
    std::vector<Vector2d> centers;

    Blob bottomYellow;
    Blob bottomBlue;


    double angleMapOdom = 0;

    /// STATE Machine run function
    ObjectParserStates run();

    /// Splits green posts
    void splitGreenPosts();

    void decreaseProbabilityCounter(std::vector<Blob> &blobs);

    void forcePucksToThree(std::vector<Blob> &blobs);

    /// convert blobs coordinates to world coordinates
    std::vector<cv::Point2f> convertToWorldPoint(std::vector<Blob> &blobs);

    /// calc distance between a blob and multiple other blobs
    double difference(std::vector<Blob> &blobs, Blob toBlob);

    /// update objects (call this frequently)
    void updateWithObjects(ImageProcessingObjects &objects);

    /// Publish TF to show all objects in RVIZ
    void pubTF();

    Vector2d getTwoCloseGreenPosts();
    Vector2d getMiddleOfGreenPosts();
    Vector2d getMiddleOf(std::vector<Blob> &blobs) ;

    void getClosestBlob(std::vector<Blob> &blobs);

    void getClosestPuck(std::vector<Blob> &blobs);

    void getBiggestBlob(std::vector<Blob> &blobs);

    static Blob getClosestBlobFrom(std::vector<Blob> &blobs) {
        Blob res;
        res.meanDistance = 10;
        for (Blob b : blobs) {
            if (b.meanDistance < res.meanDistance) {
                res = b;
                ROS_INFO_STREAM("closest " << res.meanDistance);
            }
        }
        if (res.meanDistance > 9) {
            res.meanDistance = 0;
        }
        return res;
    }

    static std::vector<Blob> filterBlobsWith(std::vector<Blob> &blobs, Blob blob) {
        std::vector<Blob> res;
        for (Blob b : blobs) {
            if (b.meanDistance < blob.meanDistance) {
                res.push_back(b);
            }
        }
        return res;
    }
};

#endif // OBJECT_PARSER_H

