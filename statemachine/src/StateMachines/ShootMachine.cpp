
#include "ShootMachine.h"
#include "roboControl.h"
#include "angles/angles.h"
using namespace std;

ShootStates ShootMachine::run() {
    static int tryGetPuck = 0;
    static int waitForStop = 0;
    if (waitForStop > 0) { waitForStop--; }
    Blob closest;
    ImageProcessingObjects objects = imageProcessing_->getObjects();
    vector<Blob> pucks = objects.yellowPucks;
//    if ((objects.blackRobot.size() > 0) &&( waitForStop < 2)){
//        ROS_INFO_STREAM("DETEECTED ROBOT");
//        waitForStop = 1/frequency_ * 18;
//        changeState(ShootStates::DETECTED_ROBOT);
//    }

    SM_DURING; /// DEFINE DURING STATE
    /// ERROR:
    case ShootStates::DETECTED_ROBOT:

        SM_DO_UNTIL(3) {
            rob_->move(-0.2,0.);
        }
        SM_AFTER(4.) {
            changeState(lastState_);
        }
        break;


    case ShootStates::START:
        changeState(ShootStates::DRIVE_TO_BACK_HOME);
        break;
    case ShootStates::DRIVE_TO_BACK_HOME:
        tryGetPuck = 0;
        if (rob_->move(Vector2d(rob_->poseInit_), 0.42, 0.1)) {
            changeState(ShootStates::TURN);
        }
        SM_AFTER(14) {
            changeState(ShootStates::TURN);
        }
        break;
    case ShootStates::DRIVE_LITTLE_BACK: {
        SM_DO_UNTIL(4) {
           rob_->move(-0.0,-0.1);
        }
        SM_AFTER(4) {
            SM_DO_UNTIL(6) rob_->move(-0.3,0);
        }
        SM_AFTER(6) {
            rob_->move(0.3,0);
        }
        SM_AFTER(8) {
            changeState(ShootStates::TURN);
        }
        break;
    }
    case ShootStates::TURN:
        rob_->move(-0.4,0);

        SM_AFTER(7) {
            changeState(ShootStates::FIND_PUCK);
        }
        break;
    case ShootStates::FIND_PUCK: {
        double angle = imageProcessing_->isPuckAtRobot();
        if (angle < 999) {
            rob_->move(angles::from_degrees(angle), 0.01);
        } else {
            if (rob_->move(Vector2d(shootPuck.worldPosition),0.3,0.08)) {
                changeState(ShootStates::FIND_PUCK_DETAIL);
            }
        }
        SM_AFTER(25) {
            ROS_INFO_STREAM("closestPuck  " << shootPuck.worldPosition << "  ");
            changeState(ShootStates::FIND_PUCK_DETAIL);
        }
        break;
    }

    case ShootStates::FIND_PUCK_EX: {
        double angle = imageProcessing_->isPuckAtRobot();
        SM_DO_UNTIL(2){
            if (angle < 999)
                rob_->move(angles::from_degrees(angle), 0.1);
        } else {
            changeState(ShootStates::FIND_PUCK);
        }
        break;
    }

    case ShootStates::FIND_PUCK_DETAIL:
        if (imageProcessing_->isPuckAtRobot() < 999) {
            changeState(ShootStates::DRIVE_TO_GOAL);
        } else {
            changeState(ShootStates::FIND_RECALIBRATE);
        }
        SM_AFTER(10) {
            changeState(ShootStates::FIND_RECALIBRATE);
        }
        break;
    case ShootStates::FIND_RECALIBRATE: {
        double backward = 0;
        SM_DO_UNTIL(2) {
            backward = -0.1;
        } else {
            double angle = imageProcessing_->isPuckAtRobot();
            backward = 0.1;
            if (angle < 998) {
                rob_->move(angles::from_degrees(angle),backward);
                if(fabs(angle) < 3) {
                    changeState(ShootStates::DRIVE_FORWARD);
                }
            } else {
                tryGetPuck++;
                if (tryGetPuck > 2) {
                    changeState((ShootStates::DRIVE_TO_BACK_HOME));
                } else if (tryGetPuck > 1){
                    changeState(ShootStates::DRIVE_RANDOM);
                }else {
                    changeState(ShootStates::DRIVE_LITTLE_BACK);
                }
            }
        }
        break;}
    case ShootStates::DRIVE_FORWARD:
        SM_DO_UNTIL(1) {
            rob_->move(0.,0.09);
        } else {
            changeState(ShootStates::DRIVE_TO_GOAL);
        }
        SM_AFTER(2) {
            changeState(ShootStates::DRIVE_TO_GOAL);
        }
        break;

    case ShootStates::DRIVE_TO_PUCK:
        if (rob_->move(aim, 0.2, 0.05) == true) {
            changeState(ShootStates::DROP_PUCK);
        }
        SM_AFTER(20) {
            changeState(ShootStates::DROP_PUCK);
        }
        break;
    case ShootStates::DRIVE_TO_GOAL:
        if (imageProcessing_->isPuckAtRobot() > 998) {
            changeState(ShootStates::FIND_RECALIBRATE);
        }
        if (rob_->driveToGoal()) {
            changeState(ShootStates::DROP_PUCK);
        }
        SM_AFTER(30) {
            changeState(ShootStates::FIND_RECALIBRATE);
        }
        break;
    case ShootStates::DROP_PUCK:
        SM_DO_UNTIL(1.4) {
            rob_->move(0, -0.2);
        } else {
            if (imageProcessing_->isYellowTeam_) {
                vector<Blob> blobs;
                for(Blob b : objParser_->yellowPucks.blobs) {
                    if (b == shootPuck) continue;
                    blobs.push_back(b);
                }
                objParser_->yellowPucks.blobs.clear();
                objParser_->yellowPucks.blobs = blobs;
            } else {
                vector<Blob> blobs;
                for(Blob b : objParser_->bluePucks.blobs) {
                    if (b == shootPuck) continue;
                    blobs.push_back(b);
                }
                objParser_->bluePucks.blobs.clear();
                objParser_->bluePucks.blobs = blobs;
            }
            shootedPucks++;
            changeState(ShootStates::DRIVE_TO_BACK_HOME);
        }
        break;
    case ShootStates::DRIVE_TO_PUCK_CLOSE:{
        double angleDiff = imageProcessing_->isPuckAtRobot();
        if ((int)angleDiff == 999) {
            changeState(ShootStates::DRIVE_TO_PUCK);
        }
        break;
    }
    case ShootStates::DRIVE_RANDOM:{
        if (rob_->move(aim, 0.24,0.1)) {
            changeState(ShootStates::TURN);
        }
        SM_AFTER(8) {
            changeState(ShootStates::TURN);
        }

        break;}
    case ShootStates::END:
        break;

    SM_EXIT; /// DEFINE EXIT STATE HERE
    SM_ENTRY; /// DEFINE ENTRY STATE

    case ShootStates::START:
        ROS_INFO_STREAM("Shootmachine started!");
        break;
    case ShootStates::TURN:
        ROS_INFO_STREAM("TURN started!");
        break;
    case ShootStates::DRIVE_TO_BACK_HOME:
        ROS_INFO_STREAM("DRIVE_TO_BACK_HOME started!");
        break;
    case ShootStates::FIND_PUCK:
        if (imageProcessing_->isYellowTeam_) {
            if (objParser_->yellowPucks.blobs.size() > 0) {
                for(Blob b : objParser_->yellowPucks.blobs) {
                    if(b.isInGoal == false) {
                        shootPuck = b;
                        break;
                    }
                }
            }
        } else {
            if (objParser_->bluePucks.blobs.size() > 0) {
                for(Blob b : objParser_->bluePucks.blobs) {
                    if(b.isInGoal == false) {
                        shootPuck = b;
                        break;
                    }
                }
            }
        }
        ROS_INFO_STREAM("Shootmachine FINDPUCK!");
        break;
    case ShootStates::FIND_PUCK_EX:
        ROS_INFO_STREAM("Shootmachine FIND_PUCK_EX!");
        break;

    case ShootStates::DRIVE_TO_PUCK:
        if (goals.size() >= 3) {
            aim = goals[shootedPucks];
        } else {
            switch(shootedPucks) {
            case 0:
                aim = Vector2d(rob_->poseInit_.position.x + 2 * a, rob_->poseInit_.position.y);
                break;
            case 1:
                aim = Vector2d(rob_->poseInit_.position.x + 2 * a, rob_->poseInit_.position.y - 0.05*b);
                break;
            case 2:
                aim = Vector2d(rob_->poseInit_.position.x + 2 * a, rob_->poseInit_.position.y + 0.05 *b);
                break;
            }


        }
        ROS_INFO_STREAM("Shootmachine DRIVE_TO_PUCK!");
        break;
    case ShootStates::DRIVE_TO_GOAL:
        ROS_INFO_STREAM("Shootmachine DRIVE_TO_GOAL!");
        break;

    case ShootStates::DROP_PUCK:
        ROS_INFO_STREAM("Shootmachine DROP_PUCK!");
        break;
    case ShootStates::DRIVE_FORWARD:
        ROS_INFO_STREAM("Shootmachine DRIVE_FORWARD!");
        break;
    case ShootStates::DRIVE_RANDOM: {
        ROS_INFO_STREAM("DRIVE RANDOM");
        double x = (rand() % 100) * 0.01 * a + a;
        double y = (rand() % 100) * 0.01 * 0.6 * b + 0.2 * b;
        aim = Vector2d(x,y);
        break;}

    SM_END;

    return currentState_;
}
