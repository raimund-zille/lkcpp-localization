#include "ObjectParser.h"
#include "image_processing.h"
#include <algorithm>
#include "roboControl.h"

using namespace std;

ObjectParserStates ObjectParser::run() {
    ImageProcessingObjects objects = imageProcessing_->getObjects();
    updateWithObjects(objects);
    SM_DURING;
    case ObjectParserStates::START:
        changeState(ObjectParserStates::GET_OBJECTS);
        break;

    case ObjectParserStates::GET_OBJECTS:{
        splitGreenPosts();
        SM_DO_IN_INTERVAL(2) {
            imageProcessing_->calcKmeans(greenPosts, centers);
            decreaseProbabilityCounter(greenPosts);
            decreaseProbabilityCounter(yellowPucks.blobs);
            decreaseProbabilityCounter(bluePucks.blobs);
            forcePucksToThree(yellowPucks.blobs);
            forcePucksToThree(bluePucks.blobs);
            //pubTF();

            //splitGreenPosts();
        }
        //
        SM_AFTER(10) {
           // splitGreenPosts();
            //changeState(ObjectParserStates::PARSING_OBJECTS);
        }

        break;
    }
    case ObjectParserStates::PARSING_OBJECTS:{

        changeState(ObjectParserStates::GET_OBJECTS);

        break;
    }
    SM_EXIT;
    SM_ENTRY;
    SM_END;
    return currentState_;
}

Vector2d ObjectParser::getTwoCloseGreenPosts() {
    double minimumDistance = 10;
    Vector2d out(0.,0.);
    Blob *used, *used2;
    for(Blob &b : greenPosts) {
        for(Blob &p : greenPosts) {
            LineSegment l = LineSegment(b.worldPosition, p.worldPosition);
            if (l.getLength() < minimumDistance) {
                used = &b;
                used2 = &p;
                out = l.getMiddleVector();
            }
        }
    }
    if (minimumDistance < 9) {
        used->alreadyUsedToDrive = true;
        used2->alreadyUsedToDrive = true;
    }
    return out;
}

Vector2d ObjectParser::getMiddleOfGreenPosts() {
    return (getMiddleOf(greenPostsSide1) + getMiddleOf(greenPostsSide2)) / 2;
}

Vector2d ObjectParser::getMiddleOf(vector<Blob> &blobs) {
    Vector2d center = Vector2d(0.,0.);
    for(Blob b : blobs) {
        center.x += b.worldPosition.position.x;
        center.y += b.worldPosition.position.y;
    }
    center = center / (blobs.size() + blobs.size());
    return center;
}

void ObjectParser::splitGreenPosts() {
    Line l;
    if (centers.size() >= 2)  {
//        if (leftLineIsSet) {
//            l = Line(LineSegment(centers[0], centers[1]).getMiddleVector(), leftLine.getDirectionVector());
//        } else {
            l = Line(Vector2d(rob_->poseInit_), LineSegment(centers[0], centers[1]).getMiddleVector());
//        }
    } else {
        l = Line(Vector2d(rob_->poseInit_), puckCenter);
    }
    int use_Points = 4;
    std::vector<Blob> t1;
    std::vector<Blob> t2;
    greenPostsSide1.clear();
    greenPostsSide2.clear();
    for(Blob &b : greenPosts) {
        if (b.side == 1) {
            t1.push_back(b);
        } else {
            t2.push_back(b);
        }
    }
    sort(t1.begin(), t1.end(), compareByProbabiliy);
    sort(t2.begin(), t2.end(), compareByProbabiliy);
    int i = 0;
    for(Blob &b : t1) {
        if (i>=use_Points) {
            break;
        }
        //b.probabilityCounter += 10;
        greenPostsSide1.push_back(b);
        i++;
    }
    i = 0;
    for(Blob &b : t2) {
        if (i>=use_Points) {
            break;
        }
        //b.probabilityCounter += 10;
        greenPostsSide2.push_back(b);
        i++;
    }
    if ((greenPostsSide1.size() >= 4) && (greenPostsSide2.size() >= 4)){
        greenPostsSplitted = true;
    }
    //ROS_INFO_STREAM(puckCenter << "  " << l);
    bool greenPosts1IsLeft = false;
    for(Blob &b : greenPostsSide1) {
        if (l.isLeftOfLine(b.worldPosition)) {
            b.isLeftSide = true;
            greenPosts1IsLeft = true;
        }
    }
    for(Blob &b : greenPosts) {
        if (l.isLeftOfLine(b.worldPosition)) {
            b.isLeftSide = true;
        }
    }
    imageProcessing_->drawLine(imageProcessing_->image_to_draw_, l, cv::Scalar(0,0,255));
    imageProcessing_->drawVector2d(imageProcessing_->image_to_draw_, puckCenter, cv::Scalar(0,0,255));
    for(Blob &b : greenPostsSide2) {
        if (l.isLeftOfLine(b.worldPosition)) {
            b.isLeftSide = true;
        }
    }
    Line dir;
    greenPostsLeft.clear();
    greenPostsRight.clear();
    if (greenPosts1IsLeft) {
        for(Blob &b : greenPostsSide1) {
            greenPostsLeft.push_back(b);
            int count = 0;
            for(Blob &p : greenPostsSide1) {
                if (b == p) continue;
                if (Line(b.worldPosition, p.worldPosition).getDirectionVector() * l.getDirectionVector() > 0) {
                    count++;
                }
            }
            if (count == 3) {
                leftBackCenter_ = b;
                break;
            }
        }
        for(Blob &b : greenPostsSide2) {
            greenPostsRight.push_back(b);
        }
        for(Blob &b : greenPostsSide1) {
            if (b == leftBackCenter_) continue;
            dir = Line(leftBackCenter_.worldPosition, b.worldPosition);
            imageProcessing_->drawLine(imageProcessing_->image_to_draw_,  dir, cv::Scalar(0,0,255));
        }
        for(Blob &b : greenPostsSide1) {
            if (b == leftBackCenter_) continue;
            dir.adjustDirection(Line(leftBackCenter_.worldPosition, b.worldPosition));
        }
    } else {
        for(Blob &b : greenPostsSide2) {
            greenPostsLeft.push_back(b);
            int count = 0;
            for(Blob &p : greenPostsSide2) {
                if (b == p) continue;
                if (Line(b.worldPosition, p.worldPosition).getDirectionVector() * l.getDirectionVector() > 0) {
                    count++;
                }
            }
            if (count == 3) {
                leftBackCenter_ = b;
                break;
            }
        }
        for(Blob &b : greenPostsSide1) {
            greenPostsRight.push_back(b);
        }
        for(Blob &b : greenPostsSide2) {
            if (b == leftBackCenter_) continue;
            dir = Line(leftBackCenter_.worldPosition, b.worldPosition);
            imageProcessing_->drawLine(imageProcessing_->image_to_draw_,  dir, cv::Scalar(0,0,255));
        }
        for(Blob &b : greenPostsSide2) {
            if (b == leftBackCenter_) continue;
            dir.adjustDirection(Line(leftBackCenter_.worldPosition, b.worldPosition));
        }
    }
    leftLine = dir;
    imageProcessing_->drawLine(imageProcessing_->image_to_draw_,  leftLine, cv::Scalar(255,0,0));
    imageProcessing_->drawVector2d(imageProcessing_->image_to_draw_, Vector2d(leftBackCenter_.worldPosition), cv::Scalar(0,0,255));
    leftLineIsSet = true;

//    greenPosts.clear();
//    greenPosts.insert(greenPosts.end(), greenPostsSide1.begin(), greenPostsSide1.end());
//    greenPosts.insert(greenPosts.end(), greenPostsSide2.begin(), greenPostsSide2.end());
}


void ObjectParser::decreaseProbabilityCounter(vector<Blob> &blobs) {
    for (vector<Blob>::iterator blob=blobs.begin(); blob!=blobs.end();) {
        blob->probabilityCounter -= 1;
        if ((blob->probabilityCounter < -8) && (!blob->isInGoal)) {
            ROS_INFO_STREAM("ProbabilityCounter Blob with color: " << blob->color);
            blob = blobs.erase(blob);
        } else {
            ++blob;
        }
    }
    sort(blobs.begin(), blobs.end(), compareByProbabiliy);
}

void ObjectParser::forcePucksToThree(vector<Blob> &blobs) {
    sort(blobs.begin(), blobs.end(), compareByProbabiliy);
    if (blobs.size() >= 3) {
        blobs.erase(blobs.begin() + 3, blobs.end());
    }
}

std::vector<cv::Point2f> ObjectParser::convertToWorldPoint(std::vector<Blob> &blobs) {
    std::vector<cv::Point2f> output;
    for (Blob &b : blobs) {
        cv::Point2f p;
        p.x =  b.worldPosition.position.x;
        p.y =  b.worldPosition.position.y;
        output.push_back(p);
    }
    return output;
}

double ObjectParser::difference(std::vector<Blob> &blobs, Blob toBlob) {
    double difference = 0.29;
    int index = -1;
    int count = 0;
    for (Blob &b : blobs) {
        double dist = distanceBetweenTwoPoints(b.worldPosition, toBlob.worldPosition);
        if (difference > dist) {
            difference = dist;
            index = count;
        }
        count++;
    }
    if (index != -1) {
        blobs[index].worldPosition.position.x = (blobs[index].worldPosition.position.x + toBlob.worldPosition.position.x) / 2;
        blobs[index].worldPosition.position.y = (blobs[index].worldPosition.position.y + toBlob.worldPosition.position.y) / 2;
        blobs[index].probabilityCounter += 1;
    }
    return difference;
}

void ObjectParser::updateWithObjects(ImageProcessingObjects &objects) {
    for(Blob b : objects.greenPosts) {
        if (difference(greenPosts, b) > 0.20) {
            b.probabilityCounter += 8;
            greenPosts.push_back(b);
        }
    }
    for(Blob b : objects.bluePucks) {
        if (difference(bluePucks.blobs, b) > 0.20) {
            b.probabilityCounter += 8;
            bluePucks.blobs.push_back(b);
        }
    }
    for(Blob b : objects.yellowPucks) {
        if (difference(yellowPucks.blobs, b) > 0.20) {
            b.probabilityCounter += 8;
            yellowPucks.blobs.push_back(b);
        }
    }
    if (objects.bottomYellow.size() > 0) {
        if (bottomYellow.isInitialized == false ) {
            bottomYellow = objects.bottomYellow[0];
        } else {
            bottomYellow.worldPosition.position.x = (bottomYellow.worldPosition.position.x + objects.bottomYellow[0].worldPosition.position.x) / 2;
            bottomYellow.worldPosition.position.y = (bottomYellow.worldPosition.position.y + objects.bottomYellow[0].worldPosition.position.y) / 2;
        }
    }
    if (objects.bottomBlue.size() > 0) {
        if (bottomBlue.isInitialized == false ) {
            bottomBlue = objects.bottomBlue[0];
        } else {
            bottomBlue.worldPosition.position.x = (bottomBlue.worldPosition.position.x + objects.bottomBlue[0].worldPosition.position.x) / 2;
            bottomBlue.worldPosition.position.y = (bottomBlue.worldPosition.position.y + objects.bottomBlue[0].worldPosition.position.y) / 2;
        }
    }
    puckCenter = Vector2d(0.,0.);
    for (Blob b : yellowPucks.blobs) {
        puckCenter.x += b.worldPosition.position.x;
        puckCenter.y += b.worldPosition.position.y;
    }
    for (Blob b : bluePucks.blobs) {
        puckCenter.x += b.worldPosition.position.x;
        puckCenter.y += b.worldPosition.position.y;
    }
    puckCenter =  puckCenter / (double)(bluePucks.blobs.size() + yellowPucks.blobs.size());
}

void ObjectParser::pubTF() {
    int i = 0;
    tf::Transform transform;
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    if (bottomBlue.isInitialized) {
        transform.setOrigin( tf::Vector3(bottomBlue.worldPosition.position.x, bottomBlue.worldPosition.position.y, 0.0) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "bottomBlue"));
    }
    if (bottomYellow.isInitialized) {
        transform.setOrigin( tf::Vector3(bottomYellow.worldPosition.position.x, bottomYellow.worldPosition.position.y, 0.0) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "bottomYellow"));
    }

//    q = tf::createQuaternionFromYaw(angleMapOdom);
//    transform.setRotation(q);
//    transform.setOrigin( tf::Vector3(0, 0, 0.0) );
//    //ROS_INFO_STREAM("PUB TF");
//    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "map"));


    for(Blob &b : greenPosts) {
        i++;
        tf::Pose pose;
        tf::poseMsgToTF(b.worldPosition, pose);
        transform.setRotation(pose.getRotation());
        transform.setOrigin( tf::Vector3(b.worldPosition.position.x, b.worldPosition.position.y, 0.0) );
        //ROS_INFO_STREAM("g " << greenPosts.size() << " " << b.worldPosition.position.x << "  " << b.worldPosition.position.y);
        std::ostringstream ss;
        ss << "greenPost " << i << " Side " << b.side;
        std::string t = ss.str();
        b.transformName = t;
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", t));
    }
    i=0;
    for(Blob b : bluePucks.blobs) {
        i++;
        tf::Pose pose;
        tf::poseMsgToTF(b.worldPosition, pose);
        transform.setRotation(pose.getRotation());
        transform.setOrigin( tf::Vector3(b.worldPosition.position.x, b.worldPosition.position.y, 0.0) );
        //ROS_INFO_STREAM("g " << greenPosts.size() << " " << b.worldPosition.position.x << "  " << b.worldPosition.position.y);
        std::ostringstream ss;
        ss << "bluePucks " << i;
        std::string t = ss.str();
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", t));
    }
    i=0;
    for(Blob b : yellowPucks.blobs) {
        i++;
        tf::Pose pose;
        tf::poseMsgToTF(b.worldPosition, pose);
        transform.setRotation(pose.getRotation());
        transform.setOrigin( tf::Vector3(b.worldPosition.position.x, b.worldPosition.position.y, 0.0) );
        //ROS_INFO_STREAM("g " << greenPosts.size() << " " << b.worldPosition.position.x << "  " << b.worldPosition.position.y);
        std::ostringstream ss;
        ss << "yellowPucks " << i;
        std::string t = ss.str();
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", t));
    }
}

void ObjectParser::getClosestBlob(std::vector<Blob> &blobs) {
    Blob res;
    res.meanDistance = 10;
    for (Blob b : blobs) {
        if (b.meanDistance < res.meanDistance) {
            res = b;
        }
    }
    if (res.color == ImageProcessingTypes::YELLOW) {
        if (closestYellow.meanDistance > res.meanDistance) {
            ROS_INFO_STREAM("Yellow closest " << res.meanDistance);
            closestYellow = res;
        }
    } else {
        if (closestBlue.meanDistance > res.meanDistance) {
            ROS_INFO_STREAM("Blue closest " << res.meanDistance);
            closestBlue = res;
        }
    }
}

void ObjectParser::getClosestPuck(std::vector<Blob> &blobs) {
    Blob res;
    bool update = false;
    res.medianDistance = 10;
    for (Blob b : blobs) {
        if ((b.medianDistance < closestPuck.medianDistance) && (b.medianDistance > 0.1)) {
            res = b;

            update = true;
        }
    }
    if (update) {
        ROS_INFO_STREAM("closest " << res.medianDistance << " old " << closestPuck.medianDistance);
        closestPuck = res;
    }
}


void ObjectParser::getBiggestBlob(std::vector<Blob> &blobs) {
    Blob res;
    res.contourArea = 0;
    for (Blob b : blobs) {
        if (b.contourArea > res.contourArea) {
            res = b;
        }
    }
    biggestBlob = res;
}
