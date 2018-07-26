#include <QTimer>
#include <queue>
#include "renderarea.h"

RenderArea::RenderArea(QWidget *parent) : QWidget(parent) {
  setAttribute(Qt::WA_StaticContents);
  scribbling = false;
  rrt = new RRT;
}

/**
 * @brief Draw the world.
 * @param painter
 */
void RenderArea::drawField(QPainter &painter) {
  painter.save();
  painter.setRenderHint(QPainter::Antialiasing);
  QRect field;
  field.setTopLeft(QPoint(this->x(), this->y()));
  field.setBottomRight(QPoint(this->width() - 1, this->height() - 1));
  painter.setPen(Qt::black);
  painter.setBrush(QBrush(Qt::white));
  painter.drawRect(field);
  painter.restore();
}

/**
 * @brief Draw the start position of the bot.
 * @param painter
 */
void RenderArea::drawStartPos(QPainter &painter) {
  painter.save();
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(Qt::black);
  painter.setBrush(QBrush(Qt::red));
  painter.drawEllipse(this->x() + START_POS_X - BOT_RADIUS,
                      this->y() + START_POS_Y - BOT_RADIUS, 2 * BOT_RADIUS,
                      2 * BOT_RADIUS);
  painter.restore();
}

/**
 * @brief Draw the end point.
 * @param painter
 */
void RenderArea::drawEndPos(QPainter &painter) {
  painter.save();
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(Qt::black);
  painter.setBrush(QBrush(Qt::blue));
  painter.drawEllipse(END_POS_X - BOT_RADIUS, END_POS_Y - BOT_RADIUS,
                      2 * BOT_RADIUS, 2 * BOT_RADIUS);
  painter.restore();
}

/**
 * @brief Draw all the rectangular obstacles.
 * @param painter
 */
void RenderArea::drawObstacles(QPainter &painter) {
  painter.save();
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(Qt::black);
  painter.setBrush(QBrush(Qt::black));
  pair<Vector2f, Vector2f> obstacle;
  for (int i = 0; i < (int)rrt->obstacles->obstacles.size(); i++) {
    obstacle = rrt->obstacles->obstacles[i];
    QPoint topLeft(obstacle.first.x() + BOT_CLEARANCE,
                   obstacle.first.y() + BOT_CLEARANCE);
    QPoint bottomRight(obstacle.second.x() - BOT_CLEARANCE,
                       obstacle.second.y() - BOT_CLEARANCE);
    QRect rect(topLeft, bottomRight);
    painter.drawRect(rect);
  }
  painter.restore();
}

/**
 * @brief Draw all the nodes generated in the RRT algorithm.
 * @param painter
 */
void RenderArea::drawNodes(QPainter &painter) {
  painter.save();
  painter.setRenderHint(QPainter::Antialiasing);
  Vector2f pos;

  if (rrt->reached()) {
      drawTree(painter,rrt->root);
      painter.setPen(Qt::green);
      painter.setBrush(QBrush(Qt::green));
    //      printf("RRT-STAR\n");
    for (int i = 0; i < (int)rrt->nodes.size(); i++) {
      for (int j = 0; j < (int)rrt->nodes[i]->children.size(); j++) {
        pos = rrt->nodes[i]->children[j]->position;
        painter.drawEllipse(pos.x() - 1.5, pos.y() - 1.5, 3, 3);
      }
      pos = rrt->nodes[i]->position;
      painter.drawEllipse(pos.x() - NODE_RADIUS, pos.y() - NODE_RADIUS,
                          2 * NODE_RADIUS, 2 * NODE_RADIUS);
    }
    painter.setPen(Qt::red);
    painter.setBrush(QBrush(Qt::red));

    // if a path exists, draw it.
    for (int i = 0; i < (int)rrt->path.size() - 1; i++) {
      QPointF p1(rrt->path[i]->position.x(), rrt->path[i]->position.y());
      QPointF p2(rrt->path[i + 1]->position.x(),
                 rrt->path[i + 1]->position.y());
      painter.drawLine(p1, p2);
    }
  } else {
      painter.setPen(Qt::green);
      painter.setBrush(QBrush(Qt::green));
    //      printf("RRT-STAR\n");
    for (int i = 0; i < (int)rrt->nodes1.size(); i++) {
      for (int j = 0; j < (int)rrt->nodes1[i]->children.size(); j++) {
        pos = rrt->nodes1[i]->children[j]->position;
        painter.drawEllipse(pos.x() - 1.5, pos.y() - 1.5, 3, 3);
      }
      pos = rrt->nodes1[i]->position;
      painter.drawEllipse(pos.x() - NODE_RADIUS, pos.y() - NODE_RADIUS,
                          2 * NODE_RADIUS, 2 * NODE_RADIUS);
    }
    for (int i = 0; i < (int)rrt->nodes2.size(); i++) {
      for (int j = 0; j < (int)rrt->nodes2[i]->children.size(); j++) {
        pos = rrt->nodes2[i]->children[j]->position;
        painter.drawEllipse(pos.x() - 1.5, pos.y() - 1.5, 3, 3);
      }
      pos = rrt->nodes2[i]->position;
      painter.drawEllipse(pos.x() - NODE_RADIUS, pos.y() - NODE_RADIUS,
                          2 * NODE_RADIUS, 2 * NODE_RADIUS);
    }
    painter.setPen(Qt::red);
    painter.setBrush(QBrush(Qt::red));
    // if a path exists, draw it.
    for (int i = 0; i < (int)rrt->path1.size() - 1; i++) {
      QPointF p1(rrt->path1[i]->position.x(), rrt->path1[i]->position.y());
      QPointF p2(rrt->path1[i + 1]->position.x(),
                 rrt->path1[i + 1]->position.y());
      //        printf("%lf,%lf  %lf,%lf
      //        \n",rrt->path1[i]->position.x(),rrt->path1[i]->position.y(),rrt->path1[i+1]->position.x(),rrt->path1[i+1]->position.y());
      painter.drawLine(p1, p2);
    }
    painter.setPen(Qt::blue);
    painter.setBrush(QBrush(Qt::blue));
    // if a path exists, draw it.
    for (int i = 0; i < (int)rrt->path2.size() - 1; i++) {
      QPointF p1(rrt->path2[i]->position.x(), rrt->path2[i]->position.y());
      QPointF p2(rrt->path2[i + 1]->position.x(),
                 rrt->path2[i + 1]->position.y());
      painter.drawLine(p1, p2);
    }
  }
  painter.restore();
}

void RenderArea::drawTree(QPainter &painter,shared_ptr<Node> root) {
//    printf("draw tree\n");
    painter.setPen(Qt::lightGray);
    painter.setBrush(QBrush(Qt::lightGray));
    if(root->children.empty())
    {
        return;
    }else {
        for(auto iter=root->children.begin();iter!=root->children.end();iter++)
        {
    QPointF p1(root->position.x(),root->position.y());
    QPointF p2((*iter)->position.x(),(*iter)->position.y());
painter.drawLine(p1, p2);
drawTree(painter,(*iter));
        }

    }

}


void RenderArea::paintEvent(QPaintEvent *) {
  QPainter painter(this);
  drawField(painter);
  drawStartPos(painter);
  drawEndPos(painter);
  drawObstacles(painter);
  drawNodes(painter);
  emit painting();
}

void RenderArea::mousePressEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    lastMouseClickedPoint = event->pos();
    scribbling = true;
  }
}

void RenderArea::mouseMoveEvent(QMouseEvent *event) {}

void RenderArea::mouseReleaseEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton && scribbling) {
    QPoint curPoint = event->pos();
    rrt->obstacles->addObstacle(
        Vector2f(lastMouseClickedPoint.x(), lastMouseClickedPoint.y()),
        Vector2f(curPoint.x(), curPoint.y()));
    update();
    scribbling = false;
  }
}
