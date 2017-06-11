#include <algorithm>
#include "rpd.h"


using namespace std;

void RPD::rdp(const path_iter begin, const path_iter end, double eps, vector<PathElement> &out){

  static int iteration = 0;
  double dmax = 0;

  path_iter idx = begin;
  iteration++;

  Vector2D first(begin->getVector());
  Vector2D last ((end-1)->getVector());

  Vector2D norm = (last-first).orthogonal().normalize();

  /* find point with greatest distance */
  for (auto i = begin + 1; i != end-1; ++i){
      Vector2D vec(i->getVector() - begin->getVector());
      double dist = std::fabs(norm.dot(vec));

      if (dist > dmax){
          idx = i;
          dmax = dist;
      }

  }

  // ROS_INFO_STREAM("MAX DIST: " << dmax);

  if (dmax >= eps){

      vector<PathElement> r1, r2;

      /* "end" points to one element after last element ~> idx + 1 is new end */
      rdp(begin,(idx+1),eps,r1);
      rdp(idx,end,eps,r2);

      /* append r1 and r2 */
      out.insert(out.end(),r1.begin(), (r1.end()-1));
      out.insert(out.end(),r2.begin(), r2.end());

   }
   else{
      out.push_back(*begin);
      out.push_back(*(end-1));
   }

  ROS_INFO_STREAM("ITERATIONS: " << iteration);
}

void RPD::approximate_path(const vector<Vector2D> &path, vector<Vector2D> &out,  double eps){

  out.clear();
  list_.clear();

  /* keep track of path order */
  for (auto i = 0; i < path.size(); i++){
      list_.push_back(PathElement(path[i],i));
  }

  vector<PathElement> approx;
  rdp(list_.begin(),list_.end(),eps,approx);


  /* sort waypoints */
  std::sort(approx.begin(),approx.end());

  /* assign waypoints to out vector */
  for (auto vec : approx){
      Vector2D v(vec.getX(),vec.getY());
      ROS_INFO_STREAM("NUMBER: " << vec.get_prio() << " " << v);
      out.push_back(v);
  }

}
