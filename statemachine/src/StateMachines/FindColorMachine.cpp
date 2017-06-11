#include "FindColorMachine.h"
#include "roboControl.h"

using namespace std;

FindColorStates FindColorMachine::run() {
    ImageProcessingObjects objects = imageProcessing_->getObjects();
    vector<Blob> pucks = objects.yellowPucks;
    static double turnSpeed = 0.3;

    SM_DURING; /// DEFINE DURING STATE

    case FindColorStates::START: {
        SM_AFTER(60) {
            changeState(FindColorStates::FIND_COLOR);
        }
        rob_->move(turnSpeed,0);
        break;}
    case FindColorStates::DRIVE_ARROUND: {
        if (rob_->move(aim,0.3,0.4)) {
            changeState(FindColorStates::DRIVE_ARROUND2);
        }
        SM_AFTER(6) {
            changeState(FindColorStates::DRIVE_ARROUND2);
        }
        break; }
    case FindColorStates::DRIVE_ARROUND2: {
        SM_DO_UNTIL(6) {
            rob_->move(aim, 0.3, 0.3);
        } else {
            rob_->move(-0.4,0);
        }
        SM_AFTER(12) {
            changeState(FindColorStates::DRIVE_TO_MIDDLE);
        }
        break; }
    case FindColorStates::DRIVE_TO_MIDDLE: {
        if (rob_->move(aim, 0.4, 0.15)) {
            changeState(FindColorStates::DRIVE_TO_LEFT);
        }
        SM_AFTER(8) {
            changeState(FindColorStates::DRIVE_TO_LEFT);
        }
        break; }
    case FindColorStates::DRIVE_TO_LEFT: {
        SM_DO_UNTIL(6) {
            rob_->move(aim, 0.3, 0.1);
        } else {
            rob_->move(0.4,0);
        }
        SM_AFTER(12) {
            changeState(FindColorStates::DRIVE_TO_RIGHT);
        }
        break;}
    case FindColorStates::DRIVE_TO_RIGHT: {
        SM_DO_UNTIL(6) {
            rob_->move(aim, 0.5, 0.12);
        } else {
            rob_->move(0.4,0);
        }
        SM_AFTER(12) {
            changeState(FindColorStates::DRIVE_TO_INIT);
        }
        break;}
    case FindColorStates::DRIVE_TO_INIT: {
        SM_DO_UNTIL(8) {
            rob_->move(aim, 0.15, 0.05);
        } else {
            rob_->move(0.4,0);
        }
        SM_AFTER(16) {
            changeState(FindColorStates::FIND_COLOR);
        }
        break;}
    case FindColorStates::FIND_COLOR: {
        double meanDistBlue = rob_->getMeanDistanceTo(objParser_->bluePucks.blobs);
        double meanDistYellow = rob_->getMeanDistanceTo(objParser_->yellowPucks.blobs);
        ROS_INFO_STREAM("Mean distance blue : " << meanDistBlue << " yellow: " << meanDistYellow);
        double closestDistBlue = rob_->getClosestDistanceTo(objParser_->bluePucks.blobs);
        double closestDistYellow = rob_->getClosestDistanceTo(objParser_->yellowPucks.blobs);
        ROS_INFO_STREAM("Mean distance blue : " << closestDistBlue << " yellow: " << closestDistYellow);
        rob_->move(turnSpeed,false);

            if (meanDistBlue < meanDistYellow) {
                teamColor_ = blue;
                imageProcessing_->isYellowTeam_ = false;
                ROS_INFO_STREAM(" BLUE! ");
            } else {
                teamColor_ = yellow;
                imageProcessing_->isYellowTeam_ = true;
                ROS_INFO_STREAM(" Yellow! ");
            }
            changeState(FindColorStates::END);
        break;}
    SM_EXIT; /// DEFINE EXIT STATE HERE
    SM_ENTRY; /// DEFINE ENTRY STATE
    case FindColorStates::START:
        ROS_INFO_STREAM("FIND COLOR MACHINE - START! ");
        turnSpeed = 0.3;
        break;
    case FindColorStates::DRIVE_ARROUND: {
        aim = objParser_->getMiddleOfGreenPosts();
        ROS_INFO_STREAM("FIND COLOR MACHINE - getMiddleOfGreenPosts! "  << aim);
        break; }
    case FindColorStates::DRIVE_ARROUND2: {
        aim = rob_->getClosestDistanceTo(objParser_->greenPosts);
        ROS_INFO_STREAM("FIND COLOR MACHINE - getClosestDistanceTo greenPosts! ");
        break; }
    case FindColorStates::DRIVE_TO_MIDDLE: {
        aim = Vector2d(rob_->poseInit_);
        ROS_INFO_STREAM("FIND COLOR MACHINE - DRIVE_TO_MIDDLE poseInit_! ");
        break; }
    case FindColorStates::DRIVE_TO_LEFT: {
        aim = objParser_->getMiddleOf(objParser_->greenPostsLeft);
        ROS_INFO_STREAM("FIND COLOR MACHINE - DRIVE_TO_LEFT! ");
        break; }
    case FindColorStates::DRIVE_TO_RIGHT: {
        aim = objParser_->getMiddleOf(objParser_->greenPostsRight);
        ROS_INFO_STREAM("FIND COLOR MACHINE - DRIVE_TO_RIGHT! ");
        break; }
    case FindColorStates::DRIVE_TO_INIT: {
        aim = Vector2d(rob_->poseInit_);
        ROS_INFO_STREAM("FIND COLOR MACHINE - DRIVE_TO_INIT! ");
        break; }
    case FindColorStates::FIND_COLOR: {
        ROS_INFO_STREAM("FIND COLOR MACHINE - FIND_COLOR! ");
        break; }

    SM_END;

    return currentState_;
}
