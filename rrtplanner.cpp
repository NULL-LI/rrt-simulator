#include "rrtplanner.h"


bool RRT_PLANNER::isCollisionFree(_type_position &p1, _type_position &p2) {
  return !(obstacles->isSegmentInObstacle(p1, p2));
}

RRT_PLANNER::RRT_PLANNER() {
  obstacles = new Obstacles;
  //  obstacles->isSegmentInObstacleCustom=&Obstacles::isSegmentInObstacle;
  //  obstacles->isSegmentInObstacleCustom=&Obstacles::isSegmentInObstacle;
  startPos = _type_position::Zero();
  endPos = _type_position::Zero();

  startPos[0] = START_POS_X;
  startPos[1] = START_POS_Y;
  endPos[0] = END_POS_X;
  endPos[1] = END_POS_Y;
  reached_flag = false;
  printf("root1\n");
  root1.reset(new Node);
  root1->parent = NULL;
  root1->position = startPos;
  root1->root = root1;
  nodes1.push_back(root1);
  lastNode1 = root1;

  printf("root2\n");
  root2.reset(new Node);
  root2->parent = NULL;
  root2->position = endPos;
  root2->root = root2;
  nodes2.push_back(root2);
  lastNode2 = root2;

  printf("root\n");
  root.reset(new Node);
  root->parent = NULL;
  root->position = startPos;
  root->cost = 0;
  lastNode = root;
  nodes.push_back(root);

  step_size = 5;
  max_iter = 10000;
  nearestNode = root;
  nearestDistance = distance(root->position, endPos);

  printf("set space range\n");
  for (int i = 0; i < SPACE_DIMENSION; i++) {
    space_range[i] = space_max_limit[i] - space_min_limit[i];
  }

  printf("rrt init fin\n");
}

void RRT_PLANNER::clearAll() {
  obstacles->obstacles.clear();
  obstacles->obstacles.resize(0);
  deleteNodes(root);
  deleteNodes(root1);
  deleteNodes(root2);
  nodes.clear();
  nodes.resize(0);
  path.clear();
  path.resize(0);
  nodes1.clear();
  nodes1.resize(0);
  path1.clear();
  path1.resize(0);
  nodes2.clear();
  nodes2.resize(0);
  path2.clear();
  path2.resize(0);
}
