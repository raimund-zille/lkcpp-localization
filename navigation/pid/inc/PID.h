#ifndef PID_H
#define PID_H
#include <cmath>
#include <algorithm>
#define EPSILON 0.01
#define MIN_VAL -(45./18.*M_PI/180.)
#define MAX_VAL  (45./18.*M_PI/180.)
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define SAT(val, min, max) (MIN(MAX(val, min), max))

template<typename T> T saturation(T val, T min, T max){
    return std::min(std::max(val,min),max);
}


class PID{

  public:

  PID(){}
  PID(double Kp, double Ki, double Kd, double min = -10, double max = 10); ///< provide PID specific values
  double get_new_val(double wantVal, double curVal, double dt);                                        ///< get new controll value
  void set_new_params(double Kp, double Ki, double Kd, double min = -10, double max = 10);///< set new parameter values

  private:
  double _Kp, _Ki, _Kd, _min, _max;

};




#endif /* PID_H */
