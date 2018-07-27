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

//template <class T>
struct Node {
    vector<shared_ptr<Node>> children;
    shared_ptr<Node>parent;
    Vector2f position;
    float cost;
    shared_ptr<Node>root;
};

class RRT
{
public:
    bool reached_flag;
    RRT();
    void initialize();
    shared_ptr<Node>getRandomNode();
    shared_ptr<Node>nearest(Vector2f point);
    shared_ptr<Node>nearest1(Vector2f point);
    shared_ptr<Node>nearest2(Vector2f point);
//    Node* shortest(Vector2f point);
    float distance(Vector2f &p, Vector2f &q);
    float cost(Vector2f &p, shared_ptr<Node>q) ;
    Vector2f newConfig(shared_ptr<Node>q, shared_ptr<Node>qNearest);
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
    Vector2f startPos, endPos;
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
