#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  renderArea = ui->renderArea;
  rrt = renderArea->rrt;
  simulated = false;
  resetFlag=false;
}

/**
 * @brief Start the simulator.
 */
void MainWindow::on_startButton_clicked() {
  if (simulated) {
    ui->statusBox->setText(tr("Please reset!"));
    renderArea->update();
    return;
  }
  simulated = true;
  // get step size and max iterations from GUI.
  rrt->setMaxIterations(ui->maxIterations->text().toInt());
  rrt->setStepSize(ui->stepSize->text().toInt());

  assert(rrt->step_size > 0);
  assert(rrt->max_iter > 0);

  for (int i = 0; i < renderArea->rrt->max_iter; i++) {
      if(resetFlag)
      {
          break;
      }
    do_rrt_connect();
    if(rrt->reached())
    {
        printf("Connected!\n");
        break;
    }

    renderArea->repaint();
    char str_tmp[100];
//    float minDistFound = rrt->nearestNode->cost;
    float startToEnd = rrt->distance(rrt->startPos, rrt->endPos);
    sprintf(str_tmp, "Iter %d \n B %.2f\n", i,
            startToEnd);
    ui->statusBox->setText(tr(str_tmp));

    QApplication::processEvents();
  }

  rrt->restoreNodes();
  for (int i = 0; i < renderArea->rrt->max_iter; i++) {
      if(resetFlag)
      {
          break;
      }
    do_rrt_star();
    char str_tmp[100];
    float minDistFound = rrt->nearestNode->cost;
    float startToEnd = rrt->distance(rrt->startPos, rrt->endPos);
    sprintf(str_tmp, "Iter %d \n F %.2f\n B %.2f\n", i, minDistFound,
            startToEnd);
    if (rrt->reached()) {
      sprintf(str_tmp, "Iter %d \n F %.2f\n B %.2f\nReached!\n", i,
              minDistFound, startToEnd);
    }
    ui->statusBox->setText(tr(str_tmp));

    QApplication::processEvents();
  }
resetFlag=false;

}

void MainWindow::do_rrt_connect() {
    shared_ptr<Node>q = rrt->getRandomNode();
        if (q) {
          shared_ptr<Node>qNearest1 = rrt->nearest1(q->position);
          if (rrt->distance(q->position, qNearest1->position) > rrt->step_size) {
            _type_position newConfig = rrt->newConfig(q, qNearest1);
            if (!rrt->obstacles->isSegmentInObstacle(newConfig,
                                                     qNearest1->position)) {
              shared_ptr<Node>qNew=shared_ptr<Node>(new Node);
              qNew->position = newConfig;
              rrt->addConnect(qNearest1, qNew);
            }
          }
          shared_ptr<Node>qNearest2 = rrt->nearest2(q->position);
          if (rrt->distance(q->position, qNearest2->position) > rrt->step_size) {
            _type_position newConfig = rrt->newConfig(q, qNearest2);
            if (!rrt->obstacles->isSegmentInObstacle(newConfig,
                                                     qNearest2->position)) {
              shared_ptr<Node>qNew=shared_ptr<Node>(new Node);
              qNew->position = newConfig;
              rrt->addConnect(qNearest2, qNew);
            }
          }
        }

        rrt->path1.clear();
        rrt->path2.clear();
        shared_ptr<Node>q1, q2;
        /*if (rrt->reached())*/ {
          q1 = rrt->lastNode1;
          q2 = rrt->lastNode2;
        }
        while (q1 != NULL) {
          rrt->path1.push_back(q1);
          q1 = q1->parent;
        }
        while (q2 != NULL) {
          rrt->path2.push_back(q2);
          q2 = q2->parent;
        }
}

void MainWindow::do_rrt_star() {
  // RRT Algorithm
  shared_ptr<Node>q = rrt->getRandomNode();
  if (q) {
    shared_ptr<Node>qShortest = rrt->nearest(q->position);
    if (rrt->distance(q->position, qShortest->position) > rrt->step_size) {
      _type_position newConfig = rrt->newConfig(q, qShortest);
      if (!rrt->obstacles->isSegmentInObstacle(newConfig,
                                               qShortest->position)) {
        shared_ptr<Node>qNew=shared_ptr<Node>(new Node);
        qNew->position = newConfig;
        rrt->add(qShortest, qNew);

        rrt->getNeighbors(rrt->lastNode);
        rrt->optimizePath(rrt->lastNode /*,neighbors*/);

        if (rrt->distance(qNew->position, rrt->endPos) < rrt->nearestDistance) {
          rrt->nearestDistance = rrt->distance(qNew->position, rrt->endPos);
          rrt->nearestNode = qNew;
        }
      }
    }

    q = rrt->nearestNode;
    rrt->path.clear();

    while (q != NULL) {
      rrt->path.push_back(q);
      q = q->parent;
    }
    renderArea->repaint();

  }
}

/**
 * @brief Delete all obstacles, nodes and paths from the simulator.
 */
void MainWindow::on_resetButton_clicked() {
    resetFlag=true;
  simulated = false;
  ui->statusBox->setText(tr(""));
  rrt->obstacles->obstacles.clear();
  rrt->obstacles->obstacles.resize(0);
  rrt->deleteNodes(rrt->root);
  rrt->deleteNodes(rrt->root1);
  rrt->deleteNodes(rrt->root2);
  rrt->nodes.clear();
  rrt->nodes.resize(0);
  rrt->path.clear();
  rrt->path.resize(0);
  rrt->nodes1.clear();
  rrt->nodes1.resize(0);
  rrt->path1.clear();
  rrt->path1.resize(0);
  rrt->nodes2.clear();
  rrt->nodes2.resize(0);
  rrt->path2.clear();
  rrt->path2.resize(0);
  rrt->initialize();
  renderArea->update();
}

MainWindow::~MainWindow() { delete ui; }
