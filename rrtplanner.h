#ifndef RRTPLANNER_H
#define RRTPLANNER_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <boost/make_shared.hpp>
#include <boost/smart_ptr.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "constants.h"
#include <rrt.h>
// #include <typeinfo.h>

using namespace std;
using namespace Eigen;
//#define Random(x) (rand() % x)


#include "obstacles.h"
class Obstacles;

class RRT_PLANNER:public RRT {
 public:
  Obstacles *obstacles;
  bool isCollisionFree(_type_position &p1, _type_position &p2);
  void clearAll();
  RRT_PLANNER();
};

#endif  // RRT_H
