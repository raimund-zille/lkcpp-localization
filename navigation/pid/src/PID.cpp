#include <ros/ros.h>
#include "../inc/PID.h"

PID::PID(double Kp, double Ki, double Kd, double min, double max):\
    _Kp(Kp),\
    _Ki(Ki),\
    _Kd(Kd),\
    _min(min),\
    _max(max)
{

}

void PID::set_new_params(double Kp, double Ki, double Kd, double min, double max){

    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;

    _min = min;
    _max = max;
}

double PID::get_new_val(double wantVal, double curVal, double dt){

    static double _i = 0, prev_err = 0;

    double err = wantVal - curVal;

    if (fabs(err) > EPSILON) _i += _Ki*err*dt;

    double _p = _Kp*err;
    double _d = _Kd*(err - prev_err)/dt;
    prev_err = err;

   // ROS_INFO_STREAM("p: " << _p << " i: " << _i << " d: " << _d);
   // ROS_INFO_STREAM("SAT: " << saturation(_p + _i +_d,_min,_max));
    return saturation(_p + _i + _d, _min, _max);


}
