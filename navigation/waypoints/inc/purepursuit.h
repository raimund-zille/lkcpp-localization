#ifndef PURE_PURSUIT
#define PURE_PURSUIT

#include <cmath>

#include "particle.h"
#include "vector2d.h"

/* see turtlebot specs  */
#define TURTLEBOT_VEL_MAX_MPS .65
#define TURTLEBOT_ANG_MAX_RPS  M_PI

struct control{
  double v;
  double w;
};

class PurePursuit {
public:

  PurePursuit() = default;
  control get_control(const pose_t &pos, const pose_t  &goal);
  bool reached_goal(double thresh);

private:

  Vector2D diff_;

};

#endif
