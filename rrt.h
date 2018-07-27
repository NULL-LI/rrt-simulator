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

template <class T>
struct Node {
    vector<shared_ptr<Node<T>>> children;
    shared_ptr<Node<T>>parent;
    T position;
    float cost;
    shared_ptr<Node<T>>root;
};
template <class T>
class RRT
{
public:
    bool reached_flag;
    RRT();
    void initialize();
    shared_ptr<Node<T>>getRandomNode();
    shared_ptr<Node<T>>nearest(Vector2f point);
    shared_ptr<Node<T>>nearest1(Vector2f point);
    shared_ptr<Node<T>>nearest2(Vector2f point);
//    Node* shortest(Vector2f point);
    float distance(T &p, T &q);
    float cost(T &p, shared_ptr<Node<T>>q) ;
    T newConfig(shared_ptr<Node<T>>q, shared_ptr<Node<T>>qNearest);
    void add(shared_ptr<Node<T>>qNearest, shared_ptr<Node<T>>qNew);
    void addConnect(shared_ptr<Node<T>>qNearest, shared_ptr<Node<T>>qNew);
    bool reached();
    void setStepSize(int step);
    void setMaxIterations(int iter);
    void deleteNodes(shared_ptr<Node<T>>root);
    Obstacles *obstacles;
    vector<shared_ptr<Node<T>>> nodes,neighborNodes;
    vector<shared_ptr<Node<T>>> path;
    shared_ptr<Node<T>>root, lastNode;
    Vector2f startPos, endPos;
    int max_iter;
    int step_size;    
    void getNeighbors(shared_ptr<Node<T>>q);
    void optimizePath(shared_ptr<Node<T>>q/*, vector<shared_ptr<Node<T>>> neighbors*/) ;
    shared_ptr<Node<T>>nearestNode;
    float nearestDistance;
    float NEIHOOD_SIZE;
     void costBiasAndCheck(shared_ptr<Node<T>>q,float bias);
     vector<shared_ptr<Node<T>>> nodes1,nodes2;
     vector<shared_ptr<Node<T>>> path1,path2;
     shared_ptr<Node<T>>root1, lastNode1 ,root2, lastNode2;
     bool restoreNodes();
};

#endif // RRT_H
