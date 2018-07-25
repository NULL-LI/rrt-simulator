#include "rrt.h"

RRT::RRT() {
  obstacles = new Obstacles;
  startPos.x() = START_POS_X;
  startPos.y() = START_POS_Y;
  endPos.x() = END_POS_X;
  endPos.y() = END_POS_Y;
  root = new Node;
  root->parent = NULL;
  root->position = startPos;
  root->distance = 0;
  lastNode = root;
  nodes.push_back(root);
  step_size = 3;
  max_iter = 3000;
  nearestNode = root;
  nearestDistance = distance(root->position, endPos);
}

/**
 * @brief Initialize root node of RRT.
 */
void RRT::initialize() {
  root = new Node;
  root->parent = NULL;
  root->position = startPos;
  root->distance = 0;
  lastNode = root;
  nodes.push_back(root);
  nearestNode = root;
  nearestDistance = distance(root->position, endPos);
}

/**
 * @brief Generate a random node in the field.
 * @return
 */
Node *RRT::getRandomNode() {
  Node *ret;
  Vector2f point(drand48() * WORLD_WIDTH, drand48() * WORLD_HEIGHT);
  if (drand48() > 0.9) {
    point = endPos;
  }

  if (point.x() >= 0 && point.x() <= WORLD_WIDTH && point.y() >= 0 &&
      point.y() <= WORLD_HEIGHT) {
    ret = new Node;
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

float RRT::cost(Vector2f &p, Node *q) {
  Vector2f v = p - q->position;
  return sqrt(powf(v.x(), 2) + powf(v.y(), 2)) + q->distance;
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point
 * @return
 */
Node *RRT::nearest(Vector2f point) {
  float minDist = 1e9;
  Node *closest = NULL;
  for (int i = 0; i < (int)nodes.size(); i++) {
    float dist = distance(point, nodes[i]->position);
    if (dist < minDist) {
      minDist = dist;
      closest = nodes[i];
    }
  }
  return closest;
}

Node *RRT::shortest(Vector2f point) {
  float minDist = 1e9;
  Node *shortest = NULL;
  for (int i = 0; i < (int)nodes.size(); i++) {
    float dist = cost(point, nodes[i]);
    if (dist < minDist) {
      minDist = dist;
      shortest = nodes[i];
    }
  }
  return shortest;
}

/*vector<Node *>*/ void RRT::getNeighbors(Node *q) {
  //  vector<Node *> neighborNodes;
  neighborNodes.clear();
  for (int i = 0; i < (int)nodes.size(); i++) {
    float dist = distance(q->position, nodes[i]->position);
    if (dist < NEIHOOD_SIZE) {
      neighborNodes.push_back(nodes[i]);
    }
  }
  return;
}

void RRT::costBiasAndCheck(Node *q, double bias) {
  q->distance += bias;
  if (q->children.size() != 0) {
    vector<Node *>::iterator it = q->children.begin();
    for (; it != q->children.end();) {
      costBiasAndCheck(*it, bias);
      if (((*it)->parent) != q) {
        perror("Parent relationship error\n");
      }
      if (((*it)->distance) < q->distance) {
        printf("%f %f", ((*it)->distance), q->distance);
        printf("Tree cost error\n");
      }
      it++;
    }
  }
  return;
}

void RRT::optimizePath(Node *q /*,  vector<Node *> neighbors*/) {
  for (int i = 0; i < (int)neighborNodes.size(); i++) {
    float disti = distance(q->position, neighborNodes[i]->position);
    if (disti + neighborNodes[i]->distance < q->distance) {
      vector<Node *>::iterator it = q->parent->children.begin();
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
      double biasCostQ = disti + neighborNodes[i]->distance - q->distance;
      costBiasAndCheck(q, biasCostQ);
      q->parent = neighborNodes[i];
      q->distance = disti + neighborNodes[i]->distance;
      neighborNodes[i]->children.push_back(q);
    }

    for (int j = 0; j < (int)neighborNodes.size(); j++) {
      if (i == j) {
        continue;
      } else {
        float distj = distance(q->position, neighborNodes[j]->position);

        if (disti + distj + neighborNodes[i]->distance <
            neighborNodes[j]->distance) {
          vector<Node *>::iterator it =
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
          double biasCostJ = disti + distj + neighborNodes[i]->distance -neighborNodes[j]->distance;
          costBiasAndCheck(neighborNodes[j], biasCostJ);
          neighborNodes[j]->parent = q;
          neighborNodes[j]->distance =
              disti + distj + neighborNodes[i]->distance;
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
Vector2f RRT::newConfig(Node *q, Node *qNearest) {
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
void RRT::add(Node *qNearest, Node *qNew) {
  qNew->parent = qNearest;
  qNew->distance =
      qNearest->distance + distance(qNew->position, qNearest->position);
  qNearest->children.push_back(qNew);
  nodes.push_back(qNew);
  lastNode = qNew;
}

/**
 * @brief Check if the last node is close to the end position.
 * @return
 */
bool RRT::reached() {
  if (distance(lastNode->position, endPos) < END_DIST_THRESHOLD) return true;
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
void RRT::deleteNodes(Node *root) {
  for (int i = 0; i < (int)root->children.size(); i++) {
    deleteNodes(root->children[i]);
  }
  delete root;
}
