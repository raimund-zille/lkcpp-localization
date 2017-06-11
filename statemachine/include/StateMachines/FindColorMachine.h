#ifndef FINDCOLORMACHINE_H
#define FINDCOLORMACHINE_H
//ROS headers
#include "ros/ros.h"
#include "StateMachine.h"
#include "states.h"
#include "ObjectParser.h"
#include "image_processing.h"
#include "referee.h"


class RoboControl;

class FindColorMachine : public StateMachine<FindColorStates>
{
public:
    ImageProcessing *imageProcessing_;
    RoboControl     *rob_;
    ObjectParser    *objParser_;
    TeamColor teamColor_;

    Vector2d aim;
    FindColorMachine(FindColorStates initState, double frequenzy) : StateMachine<FindColorStates>(initState, frequenzy) {}

    FindColorStates run();
    void reset() {
        currentState_ = FindColorStates::START;
    }
};


#endif // SHOOTMACHINE_H
