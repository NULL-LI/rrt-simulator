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
    rrt->do_rrt_connect();
    if(rrt->reached())
    {
        printf("Connected!\n");
        break;
    }
    renderArea->repaint();
    QApplication::processEvents();

    char str_tmp[100];
//    float minDistFound = rrt->nearestNode->cost;
    float startToEnd = rrt->distance(rrt->startPos, rrt->endPos);
    sprintf(str_tmp, "Iter %d \n B %.2f\n", i,
            startToEnd);
    ui->statusBox->setText(tr(str_tmp));

  }

  rrt->restoreNodes();
  for (int i = 0; i < renderArea->rrt->max_iter; i++) {
      if(resetFlag)
      {
          break;
      }
    rrt->do_rrt_star();
    renderArea->repaint();
    QApplication::processEvents();

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

  }
resetFlag=false;

}

/**
 * @brief Delete all obstacles, nodes and paths from the simulator.
 */
void MainWindow::on_resetButton_clicked() {
    resetFlag=true;
  simulated = false;
  ui->statusBox->setText(tr(""));
rrt->clearAll();
  rrt->initialize();
  renderArea->update();
}

MainWindow::~MainWindow() { delete ui; }
