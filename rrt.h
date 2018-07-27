#ifndef RRT_H
#define RRT_H

#include "obstacles.h"
#include <stdlib.h>
#include<stdio.h>
#include <vector>
#include <math.h>
#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>
// #include <typeinfo.h>

using namespace std;
using namespace Eigen;

//#define Random(x) (rand() % x)

#define SPACE_DIMENSION 2

typedef Matrix<float,SPACE_DIMENSION,1> _type_position;

struct Node {
    vector<shared_ptr<Node>> children;
    shared_ptr<Node>parent;
    _type_position position;
    float cost;
    shared_ptr<Node>root;
};

class RRT
{
public:

    double space_max_limit[6]={WORLD_WIDTH,WORLD_HEIGHT,M_PI,M_PI,M_PI,M_PI};
    double space_min_limit[6]={0,0,-M_PI,-M_PI,-M_PI,-M_PI};
    double space_range[6]={WORLD_WIDTH,WORLD_HEIGHT,2*M_PI,2*M_PI,2*M_PI,2*M_PI};

    bool reached_flag;
    RRT();
    void initialize();
    shared_ptr<Node>getRandomNode();
    shared_ptr<Node>nearest(_type_position point);
    shared_ptr<Node>nearest1(_type_position point);
    shared_ptr<Node>nearest2(_type_position point);
//    Node* shortest(_type_position point);
    float distance(_type_position &p, _type_position &q);
    float cost(_type_position &p, shared_ptr<Node>q) ;
    _type_position newConfig(shared_ptr<Node>q, shared_ptr<Node>qNearest);
    void add(shared_ptr<Node>qNearest, shared_ptr<Node>qNew);
    void addConnect(shared_ptr<Node>qNearest, shared_ptr<Node>qNew);
    bool reached();
    void setStepSize(int step);
    void setMaxIterations(int iter);
    void deleteNodes(shared_ptr<Node>root);
    Obstacles *obstacles;
    vector<shared_ptr<Node>> nodes,neighborNodes;
    vector<shared_ptr<Node>> path;
    shared_ptr<Node>root, lastNode;
    _type_position startPos, endPos;
    int max_iter;
    int step_size;    
    void getNeighbors(shared_ptr<Node>q);
    void optimizePath(shared_ptr<Node>q/*, vector<shared_ptr<Node>> neighbors*/) ;
    shared_ptr<Node>nearestNode;
    float nearestDistance;
    float NEIHOOD_SIZE;
     void costBiasAndCheck(shared_ptr<Node>q,float bias);
     vector<shared_ptr<Node>> nodes1,nodes2;
     vector<shared_ptr<Node>> path1,path2;
     shared_ptr<Node>root1, lastNode1 ,root2, lastNode2;
     bool restoreNodes();
};

#endif // RRT_H
