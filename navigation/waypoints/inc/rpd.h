#include <ros/ros.h>

/* C++ Header files */
#include <memory>
#include <vector>

/* costum library */
#include "vector2d.h"



class PathElement : public Vector2D{

public:

  PathElement(double x, double y, int idx) : Vector2D(x,y),prio_(idx){}
  PathElement(const Vector2D &pos, int idx) : Vector2D(pos),prio_(idx){}

  int get_prio() const {return prio_;}
  bool operator<(const PathElement &other) const{
    return prio_ < other.prio_;
  }

private:

  int prio_;
};


typedef std::vector<PathElement>::iterator path_iter;

/**
 * @brief The RPD class implements the Ramer-Douglas-Pecker algorithm for approximating way points for a given path
 */
class RPD{

public:

  RPD():list_(){}

  /**
   * @brief approximate_path approximates path of global planner, so the pure pursuit algorithm is able
   * to follow a smaller number of waypoints
   * @param path: Path of global planner
   * @param out: approximated path
   * @param eps: User defined threshold
   */

  void approximate_path(const std::vector<Vector2D> &path, std::vector<Vector2D> &out,  double eps = .01);


private:

  std::vector<PathElement> list_;

  /**
   * @brief rdp: Ramer-Douglas-Pecker Algorithm
   * @param begin: begin of PathElement list
   * @param end: end of PathElement list
   * @param eps: User threshold
   * @param out: approximated path
   */
  void rdp(const path_iter begin, const path_iter end, double eps, std::vector<PathElement> &out);
};



