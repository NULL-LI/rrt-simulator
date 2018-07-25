#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;

#include <condition_variable>
#include <mutex>
#include <thread>

#define KeepLiveBegin(ignoreevent)               \
  {                                              \
    IgnoreEvent *kl_ie = NULL;                   \
    if (ignoreevent) kl_ie = new IgnoreEvent();  \
    std::condition_variable kl_cv;               \
    std::mutex kl_mtx;                           \
    std::unique_lock<std::mutex> kl_lck(kl_mtx); \
        std::thread kl_thread([&](){kl_mtx.lock(); kl_mtx.unlock();
#define KeepLiveEnd                                                \
  kl_cv.notify_all();                                              \
  });                     \
  kl_thread.detach();                                              \
  while (kl_cv.wait_for(kl_lck, std::chrono::milliseconds(100)) == \
         std::cv_status::timeout) {                                \
    qApp->processEvents();                                         \
  }                                                                \
  if (kl_ie != NULL) delete kl_ie;                                 \
  }

class IgnoreEvent : public QObject {
 public:
  IgnoreEvent(QObject *obj = qApp) {
    m_obj = obj;
    m_obj->installEventFilter(this);
  }
  ~IgnoreEvent() { m_obj->removeEventFilter(this); }
  bool eventFilter(QObject *obj, QEvent *event) {
    if (event->type() == QEvent::KeyPress ||
        event->type() == QEvent::MouseButtonPress) {
      event->ignore();
      return true;
    }
    return QObject::eventFilter(obj, event);
  }

 private:
  QObject *m_obj;
};

// KeepLiveTest::KeepLiveTest(QWidget *parent)
//    : QMainWindow(parent)
//{
//    ui.setupUi(this);
//    connect(ui.pushButton, &QPushButton::clicked, [this](){
//        int sum = 0;
//        KeepLiveBegin(true)
//        for (int i = 0; i < 100; i++)
//        {
//            ui.label->setText(QString::number(sum));
//            for (int j = 0; j < 10000000; j++)
//            {
//                sum += (i*j) & 3;
//            }
//            sum += i & 3;
//        }
//        KeepLiveEnd
//    });
//}

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
    }
    q = rrt->nearestNode;
    rrt->path.clear();
    while (q != NULL) {
      rrt->path.push_back(q);
      q = q->parent;
    }
    renderArea->repaint();
    if (rrt->reached()) {
      ui->statusBox->setText(tr("Reached Destination!"));
      //      break;
    }
    char str_tmp[100];
    sprintf(str_tmp, "Iter %d", i);
    //    ui->statusBox->setText(tr(str_tmp));
    QApplication::processEvents();
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
