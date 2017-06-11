#ifndef CALCABMACHINE_H
#define CALCABMACHINE_H
//ROS headers
#include "ros/ros.h"
#include "StateMachine.h"
#include "states.h"
#include "blob.h"
#include "ObjectParser.h"
#include "image_processing.h"

class RoboControl;

class CalcABMachine : public StateMachine<CalculateAB_States>
{
public:
    ImageProcessing *imageProcessing_;
    RoboControl     *rob_;
    ObjectParser    *objParser_;


    double a;
    double b;
    double a_b;
    double createAB(std::vector<Blob> &t1_sorted, std::vector<Blob> &t2_sorted);

    CalcABMachine(CalculateAB_States initState, double frequenzy) : StateMachine<CalculateAB_States>(initState, frequenzy) {}

    CalculateAB_States run();
    void reset() {
        currentState_ = CalculateAB_States::START;
    }
};


#endif // CALCABMACHINE_H
