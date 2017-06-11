#ifndef SHOOTMACHINE_H
#define SHOOTMACHINE_H
//ROS headers
#include "ros/ros.h"
#include "StateMachine.h"
#include "states.h"
#include "ObjectParser.h"
#include "image_processing.h"



class RoboControl;

class ShootMachine : public StateMachine<ShootStates>
{
public:
    ImageProcessing *imageProcessing_;
    RoboControl     *rob_;
    ObjectParser    *objParser_;
    Blob shootPuck;
    int shootedPucks = 0;
    std::vector<Vector2d> goals;
    Vector2d aim;
    ShootStates savedState;

    double a,b;
    ShootMachine(ShootStates initState, double frequenzy) : StateMachine<ShootStates>(initState, frequenzy) {}



    ShootStates run();
    void reset() {
        currentState_ = ShootStates::START;
    }
};


#endif // SHOOTMACHINE_H
