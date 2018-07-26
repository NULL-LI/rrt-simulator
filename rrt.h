#ifndef RRT_H
#define RRT_H

#include "obstacles.h"
#include <stdlib.h>
#include<stdio.h>
#include <vector>
#include <math.h>

using namespace std;
using namespace Eigen;

//#define Random(x) (rand() % x)

struct Node {
    vector<Node *> children;
    Node *parent;
    Vector2f position;
    double cost;
    Node *root;
};

class RRT
{
public:
    bool reached_flag;
    RRT();
    void initialize();
    Node* getRandomNode();
    Node* nearest(Vector2f point);
    Node* nearest1(Vector2f point);
    Node* nearest2(Vector2f point);
//    Node* shortest(Vector2f point);
    float distance(Vector2f &p, Vector2f &q);
    float cost(Vector2f &p, Node *q) ;
    Vector2f newConfig(Node *q, Node *qNearest);
    void add(Node *qNearest, Node *qNew);
    void addConnect(Node *qNearest, Node *qNew);
    bool reached();
    void setStepSize(int step);
    void setMaxIterations(int iter);
    void deleteNodes(Node *root);
    Obstacles *obstacles;
    vector<Node *> nodes,neighborNodes;
    vector<Node *> path;
    Node *root, *lastNode;
    Vector2f startPos, endPos;
    int max_iter;
    int step_size;    
    void getNeighbors(Node *q);
    void optimizePath(Node *q/*, vector<Node *> neighbors*/) ;
    Node *nearestNode;
    float nearestDistance;
    float NEIHOOD_SIZE;
     void costBiasAndCheck(Node *q,double bias);     
     vector<Node *> nodes1,nodes2;
     vector<Node *> path1,path2;
     Node *root1, *lastNode1 ,*root2, *lastNode2;
     bool restoreNodes();
};

#endif // RRT_H
