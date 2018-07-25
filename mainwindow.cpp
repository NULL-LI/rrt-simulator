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
}

/**
 * @brief Start the simulator.
 */
void MainWindow::on_startButton_clicked() {
  //    KeepLiveBegin(true)
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

  do_rrt();

  //  if (rrt->reached()) {
  //    q = rrt->lastNode;
  //  } else {
  //    // if not reached yet, then shortestPath will start from the closest
  //    node to
  //    // end point.
  //    q = rrt->nearest(rrt->endPos);
  //    ui->statusBox->setText(tr("Exceeded max iterations!"));
  //  }
  // generate shortest path to destination.
  // KeepLiveEnd
}

void MainWindow::do_rrt() {
  // RRT Algorithm
  for (int i = 0; i < renderArea->rrt->max_iter; i++) {
    Node *q = rrt->getRandomNode();
    if (q) {
      Node *qShortest = rrt->nearest(q->position);
      if (rrt->distance(q->position, qShortest->position) > rrt->step_size) {
        Vector2f newConfig = rrt->newConfig(q, qShortest);
        if (!rrt->obstacles->isSegmentInObstacle(newConfig,
                                                 qShortest->position)) {
          Node *qNew = new Node;
          qNew->position = newConfig;
          rrt->add(qShortest, qNew);

          rrt->getNeighbors(rrt->lastNode);
          rrt->optimizePath(rrt->lastNode /*,neighbors*/);

          if (rrt->distance(qNew->position, rrt->endPos) <
              rrt->nearestDistance) {
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
      char str_tmp[100];
      float minDistFound = rrt->nearestNode->distance;
      float startToEnd = rrt->distance(rrt->startPos, rrt->endPos);
      sprintf(str_tmp, "Iter %d \n F %.2f\n B %.2f\n", i, minDistFound,startToEnd);
      if (rrt->reached()) {
        sprintf(str_tmp, "Iter %d \n F %.2f\n B %.2f\nReached!\n", i, minDistFound,startToEnd);
        //      ui->statusBox->setText(tr("Reached Destination!"));
        //      break;
      }

      ui->statusBox->setText(tr(str_tmp));
      QApplication::processEvents();
    }
  }
}

/**
 * @brief Delete all obstacles, nodes and paths from the simulator.
 */
void MainWindow::on_resetButton_clicked() {
  simulated = false;
  ui->statusBox->setText(tr(""));
  rrt->obstacles->obstacles.clear();
  rrt->obstacles->obstacles.resize(0);
  rrt->deleteNodes(rrt->root);
  rrt->nodes.clear();
  rrt->nodes.resize(0);
  rrt->path.clear();
  rrt->path.resize(0);
  rrt->initialize();
  renderArea->update();
}

MainWindow::~MainWindow() { delete ui; }
