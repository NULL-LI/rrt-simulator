#ifndef OBSTACLES_H
#define OBSTACLES_H

#include <QDebug>
#include <QLine>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <assert.h>
#include "rrt.h"

using namespace Eigen;
using namespace std;

class Obstacles
{
public:
    Obstacles();
    void addObstacle(Vector2f firstPoint, Vector2f secondPoint);
    virtual bool isSegmentInObstacle(_type_position &p1, _type_position &p2);
    vector<pair<Vector2f, Vector2f> > obstacles;
};



#endif // OBSTACLES_H
