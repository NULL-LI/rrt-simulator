#include "rrt.h"

RRT::RRT() {
  obstacles = new Obstacles;
  startPos.x() = START_POS_X;
  startPos.y() = START_POS_Y;
  endPos.x() = END_POS_X;
  endPos.y() = END_POS_Y;
  reached_flag = false;

  root1 .reset(new Node);
  root1->parent = NULL;
  root1->position = startPos;
  root1->root = root1;
  nodes1.push_back(root1);
  lastNode1 = root1;
  root2.reset(new Node);
  root2->parent = NULL;
  root2->position = endPos;
  root2->root = root2;
  nodes2.push_back(root2);
  lastNode2 = root2;

  root .reset(new Node);
  root->parent = NULL;
  root->position = startPos;
  root->cost = 0;
  lastNode = root;
  nodes.push_back(root);

  step_size = 5;
  max_iter = 10000;
  nearestNode = root;
  nearestDistance = distance(root->position, endPos);
}

/**
 * @brief Initialize root node of RRT.
 */
void RRT::initialize() {
  reached_flag = false;

  root1.reset(new Node);
  root1->parent = NULL;
  root1->position = startPos;
  root1->root = root1;
  nodes1.push_back(root1);
  lastNode1 = root1;
  root2 .reset(new Node);
  root2->parent = NULL;
  root2->position = endPos;
  root2->root = root2;
  nodes2.push_back(root2);
  lastNode2 = root2;

  root .reset(new Node);
  root->parent = NULL;
  root->position = startPos;
  root->cost = 0;
  lastNode = root;
  nodes.push_back(root);
  nearestNode = root;
  nearestDistance = distance(root->position, endPos);
}

/**
 * @brief Generate a random node in the field.
 * @return
 */
shared_ptr<Node>RRT::getRandomNode() {
  Vector2f point(drand48() * WORLD_WIDTH, drand48() * WORLD_HEIGHT);
  if (drand48() > 0.9) {
    point = endPos;
  }
  // rrt star function

  if (reached()) {
    float minDistFound = nearestNode->cost;
    float distAdded = distance(point, startPos) + distance(point, endPos);
          float startToEnd = distance(startPos, endPos);
    //      printf("minDistFound %f startToEnd %f\n", minDistFound,startToEnd);
    int cnt = 0;
    int cntMax = 1000;
    do {
      cnt++;
      point(0) = drand48() * WORLD_WIDTH;
      point(1) = drand48() * WORLD_HEIGHT;
      distAdded = distance(point, startPos) + distance(point, endPos);
      // to be optimized
//          printf("minDist %f distAdded %f startToEnd %f\n", minDistFound,distAdded,startToEnd);
    } while (minDistFound < distAdded /*&&cnt<cntMax*/);
    if (cnt == cntMax) {
      return NULL;
    }
  }

  //  printf("nearestNode->distance %f\n", nearestNode->distance);
  if (point.x() >= 0 && point.x() <= WORLD_WIDTH && point.y() >= 0 &&
      point.y() <= WORLD_HEIGHT) {

      shared_ptr<Node>ret=make_shared<Node>();
    ret->position = point;
    return ret;
  }
  return NULL;
}

/**
 * @brief Helper method to find distance between two positions.
 * @param p
 * @param q
 * @return
 */
float RRT::distance(Vector2f &p, Vector2f &q) {
  Vector2f v = p - q;
  return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
}

float RRT::cost(Vector2f &p, shared_ptr<Node>q) {
  Vector2f v = p - q->position;
  return sqrt(powf(v.x(), 2) + powf(v.y(), 2)) + q->cost;
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point
 * @return
 */
shared_ptr<Node>RRT::nearest(Vector2f point) {
  float minDist = 1e9;
  shared_ptr<Node>closest = NULL;
  for (int i = 0; i < (int)nodes.size(); i++) {
    float dist = distance(point, nodes[i]->position);
    if (dist < minDist) {
      minDist = dist;
      closest = nodes[i];
    }
  }
  return closest;
}

shared_ptr<Node>RRT::nearest1(Vector2f point) {
  float minDist = 1e9;
  shared_ptr<Node>closest = NULL;
  for (int i = 0; i < (int)nodes1.size(); i++) {
    float dist = distance(point, nodes1[i]->position);
    if (dist < minDist) {
      minDist = dist;
      closest = nodes1[i];
    }
  }
  return closest;
}
shared_ptr<Node>RRT::nearest2(Vector2f point) {
  float minDist = 1e9;
  shared_ptr<Node>closest = NULL;
  for (int i = 0; i < (int)nodes2.size(); i++) {
    float dist = distance(point, nodes2[i]->position);
    if (dist < minDist) {
      minDist = dist;
      closest = nodes2[i];
    }
  }
  return closest;
}

// shared_ptr<Node>RRT::shortest(Vector2f point) {
//  float minDist = 1e9;
//  shared_ptr<Node>shortest = NULL;
//  for (int i = 0; i < (int)nodes.size(); i++) {
//    float dist = cost(point, nodes[i]);
//    if (dist < minDist) {
//      minDist = dist;
//      shortest = nodes[i];
//    }
//  }
//  return shortest;
//}

/*vector<shared_ptr<Node>>*/ void RRT::getNeighbors(shared_ptr<Node>q) {
  //  vector<shared_ptr<Node>> neighborNodes;
  neighborNodes.clear();
  for (int i = 0; i < (int)nodes.size(); i++) {
    float dist = distance(q->position, nodes[i]->position);
    if (dist < NEIHOOD_SIZE) {
      neighborNodes.push_back(nodes[i]);
    }
  }
  return;
}

void RRT::costBiasAndCheck(shared_ptr<Node> q, float bias) {
  q->cost += bias;
  if (q->children.size() != 0) {
    auto it = q->children.begin();
    for (; it != q->children.end();) {
      costBiasAndCheck(*it, bias);
      if (((*it)->parent) != q) {
        perror("Parent relationship error\n");
      }
      if (((*it)->cost) < q->cost) {
        printf("%f %f", ((*it)->cost), q->cost);
        printf("Tree cost error\n");
      }
      it++;
    }
  }
  return;
}

void RRT::optimizePath(shared_ptr<Node>q /*,  vector<shared_ptr<Node>> neighbors*/) {
  for (int i = 0; i < (int)neighborNodes.size(); i++) {
    float disti = distance(q->position, neighborNodes[i]->position);
    if (disti + neighborNodes[i]->cost < q->cost) {
      vector<shared_ptr<Node>>::iterator it = q->parent->children.begin();
      for (; it != q->parent->children.end();) {
        if ((*it) == q) {
          //删除指定元素，返回指向删除元素的下一个元素的位置的迭代器
          it = q->parent->children.erase(it);

          if (it == q->parent->children.end()) {
          }
        } else {
          //迭代器指向下一个元素位置
          ++it;
        }
      }
      float biasCostQ = disti + neighborNodes[i]->cost - q->cost;
      costBiasAndCheck(q, biasCostQ);
      q->parent = neighborNodes[i];
      q->cost = disti + neighborNodes[i]->cost;
      neighborNodes[i]->children.push_back(q);
    }

    for (int j = 0; j < (int)neighborNodes.size(); j++) {
      if (i == j) {
        continue;
      } else {
        float distj = distance(q->position, neighborNodes[j]->position);

        if (disti + distj + neighborNodes[i]->cost < neighborNodes[j]->cost) {
          vector<shared_ptr<Node>>::iterator it =
              neighborNodes[j]->parent->children.begin();
          for (; it != neighborNodes[j]->parent->children.end();) {
            if ((*it) == neighborNodes[j]) {
              //删除指定元素，返回指向删除元素的下一个元素的位置的迭代器
              it = neighborNodes[j]->parent->children.erase(it);
            } else {
              //迭代器指向下一个元素位置
              ++it;
            }
          }
          float biasCostJ =
              disti + distj + neighborNodes[i]->cost - neighborNodes[j]->cost;
          costBiasAndCheck(neighborNodes[j], biasCostJ);
          neighborNodes[j]->parent = q;
          neighborNodes[j]->cost = disti + distj + neighborNodes[i]->cost;
          q->children.push_back(neighborNodes[j]);
          //          printf("Optimize Finished \n");
        }
      }
    }
  }
  return;
}

/**
 * @brief Find a configuration at a distance step_size from nearest node to
 * random node.
 * @param q
 * @param qNearest
 * @return
 */
Vector2f RRT::newConfig(shared_ptr<Node>q, shared_ptr<Node>qNearest) {
  Vector2f to = q->position;
  Vector2f from = qNearest->position;
  Vector2f intermediate = to - from;
  intermediate = intermediate / intermediate.norm();
  Vector2f ret = from + step_size * intermediate;
  return ret;
}

/**
 * @brief Add a node to the tree.
 * @param qNearest
 * @param qNew
 */
void RRT::add(shared_ptr<Node>qNearest, shared_ptr<Node>qNew) {
  qNew->parent = qNearest;
  qNew->cost = qNearest->cost + distance(qNew->position, qNearest->position);
  qNearest->children.push_back(qNew);
  nodes.push_back(qNew);
  lastNode = qNew;
}

void RRT::addConnect(shared_ptr<Node>qNearest, shared_ptr<Node>qNew) {
  qNew->parent = qNearest;
  qNearest->children.push_back(qNew);
  if (qNearest->root == root1) {
    qNew->root = root1;
    nodes1.push_back(qNew);
    lastNode1 = qNew;
  }
  if (qNearest->root == root2) {
    qNew->root = root2;
    nodes2.push_back(qNew);
    lastNode2 = qNew;
  }
}

/**
 * @brief Check if the last node is close to the end position.
 * @return
 */
bool RRT::reached() {
  if (reached_flag == true) {
//      printf("flag reached true\n");
    return true;
  }

  if /*(distance(nearestNode->position, endPos) < END_DIST_THRESHOLD)*/ (
      distance(lastNode1->position, lastNode2->position) < END_DIST_THRESHOLD) {
    reached_flag = true;
    return true;
  }

//      printf("reached false\n");
  return false;
}

bool RRT::restoreNodes() {
  if (reached()) {
      printf("ready to restore!");
    nodes.clear();
    shared_ptr<Node>q1, q2;
    q1 = lastNode1;
    q2 = lastNode2;
    printf("nodes1!");
    while (q1 != NULL) {
      nodes.insert(nodes.begin(), q1);
      q1 = q1->parent;
    }
    printf("nodes1! %ld",nodes.size());
    printf("nodes2!");
    while (q2 != NULL) {
      nodes.push_back(q2);
      q2 = q2->parent;
    }
    printf("nodes2! %ld",nodes.size());
    for (int i = 0; i < (int)nodes.size(); i++) {
      nodes[i]->children.clear();
      if (i == 0) {
          nodes[i]->cost=0;
      }else {

          nodes[i]->cost=nodes[i-1]->cost+distance(nodes[i]->position,nodes[i-1]->position);
          nodes[i]->parent=nodes[i-1];
      }
      if (i != (int)nodes.size() - 1) {
        nodes[i]->children.push_back(nodes[i+1]);
      }
    }
    root=*(nodes.begin());
    nearestNode=*(nodes.end()-1);
    nearestDistance=distance(nearestNode->position, endPos);
    printf("nodes.size() %ld\n", nodes.size());
    printf("end %f\n", (*(nodes.end()-1))->cost);
    printf("begin %f\n", (*(nodes.begin()))->cost);
    float minDistFound = nearestNode->cost;
    float startToEnd = distance(startPos, endPos);
    printf("minDist %f startToEnd %f\n", minDistFound,startToEnd);
    return true;
  }


  printf("not reached yet");
  return false;
}

void RRT::setStepSize(int step) {
  step_size = step;
  NEIHOOD_SIZE = step_size * 5;
}

void RRT::setMaxIterations(int iter) { max_iter = iter; }

/**
 * @brief Delete all nodes using DFS technique.
 * @param root
 */
void RRT::deleteNodes(shared_ptr<Node>root) {
  for (int i = 0; i < (int)root->children.size(); i++) {
    deleteNodes(root->children[i]);
  }
//  delete root;
}
