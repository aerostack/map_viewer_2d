/*!********************************************************************************
 * \brief     It presents a 2D map on which shows the drone dynamic movement,
 *            obstacles in the environment and the road planned.             
 * \authors   John Bradley
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include "../include/object_controller.h"

ObjectController::ObjectController(QWidget* parent) : QGraphicsView(parent)
{
  setMouseTracking(true);
  scene = new QGraphicsScene(0, 0, this->size().width(), this->size().height(), this);
  this->setScene(scene);
  this->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  coordinates = new QGraphicsTextItem("");
  scene->addItem(coordinates);
  x_center = this->size().width() / 2;
  y_center = this->size().height() / 2;
}

ObjectController::~ObjectController()
{
  delete coordinates;
  delete scene;
}

void ObjectController::setMap(double x_meters, double y_meters, double x_init, double y_init)
{
  map.x_meters = x_meters;
  map.y_meters = y_meters;
  map.x_init = x_init;
  map.y_init = y_init;
  
  x_center = this->size().width() / 2;
  y_center = this->size().height() / 2;
  map.set = true;
  
  objects.drones.clear();
  objects.boxes.clear();
  objects.cylinders.clear();
  setAutoPixPerMeter();
  //setAutoMapWalls();
  reDraw();
  objects.map = map;
}

bool ObjectController::isMapSet()
{
  return map.set;
}

ObjectController::Objects ObjectController::getObjects()
{
  return objects;
}

void ObjectController::setObjects(ObjectController::Objects objects)
{
  this->objects = objects;
  map = objects.map;
  setAutoPixPerMeter();
  reDraw();
}

void ObjectController::setAutoPixPerMeter()
{
  double pix_per_meterT1 = 0;
  double pix_per_meterT2 = 0;
  pix_per_meterT1 = this->size().width() / (map.x_meters + 2);
  pix_per_meterT2 = this->size().height() / (map.y_meters + 2);
  
  if (pix_per_meterT1 < pix_per_meterT2)
  {
    pix_per_meter = pix_per_meterT1;
  }
  else
  {
    pix_per_meter = pix_per_meterT2;
  }
}

/* void ObjectController::setAutoMapWalls()
{
  addNewWall("bottom side", map.x_meters, 0.3, map.x_meters / 2 + map.x_init, map.y_init);
  addNewWall("up side", map.x_meters, 0.3, map.x_meters / 2 + map.x_init, map.y_meters + map.y_init);
  addNewWall("left side", 0.3, map.y_meters, map.x_init, map.y_meters / 2 + map.y_init);
  addNewWall("right side", 0.3, map.y_meters, map.x_meters + map.x_init, map.y_meters / 2 + map.y_init);
} */

void ObjectController::drawGuide()
{
  float y = this->size().height();
  float x = this->size().width();
  float meters_x = x / pix_per_meter;
  float meters_y = y / pix_per_meter;
  float x_map_pix = map.x_meters * pix_per_meter;
  float y_map_pix = map.y_meters * pix_per_meter;
  float coor_0_x = (x_center) - (x_map_pix / 2);
  float coor_0_y = (y_center) - (y_map_pix / 2);

  float meters_before_x = x_center / pix_per_meter;
  float meters_before_y = y_center / pix_per_meter;

  float coor_x_guide = coor_0_x - ((meters_before_x + 1) * pix_per_meter);
  float coor_y_guide = coor_0_y - ((meters_before_y + 1) * pix_per_meter);

  scene->addLine(coor_x_guide - 1, coor_y_guide - 1, coor_x_guide + 1, coor_y_guide + 1, pencil);
  scene->addLine(coor_x_guide + 1, coor_y_guide - 1, coor_x_guide - 1, coor_y_guide + 1, pencil);

  float total_x_meters = meters_x + 1 + map.x_meters;
  float total_y_meters = meters_y + 1 + map.y_meters;
  
  if (this->size().width() > this->size().height())
  {
    pix_guide = total_x_meters * pix_per_meter;
  }
  else
  {
    pix_guide = total_y_meters * pix_per_meter;
  }
  pencil.setBrush(QColor(0, 0, 0, 20));
  
  for (int i = 0; i < total_x_meters; i++)
  {
    scene->addLine(coor_x_guide + i * pix_per_meter, 0, coor_x_guide + i * pix_per_meter, pix_guide, pencil);
    for (int j = 0; j < 4; j++)
    {
      int pix = pix_per_meter / 4;
      scene->addLine(coor_x_guide + i * pix_per_meter + j * pix, 0, coor_x_guide + i * pix_per_meter + j * pix, pix_guide, pencil);
    }
  }
  
  for (int i = 0; i < total_y_meters; i++)
  {
    scene->addLine(0, coor_y_guide + i * pix_per_meter, pix_guide, coor_y_guide + i * pix_per_meter, pencil);
    for (int j = 0; j < 4; j++)
    {
      int pix = pix_per_meter / 4;
      scene->addLine(0, coor_y_guide + i * pix_per_meter + j * pix, pix_guide, coor_y_guide + i * pix_per_meter + j * pix, pencil);
    }
  }
  pencil.setBrush(Qt::black);
  // Code to see the center
  scene->addLine(x_center - 1, y_center - 1, x_center + 1, y_center + 1, pencil);
  scene->addLine(x_center + 1, y_center - 1, x_center - 1, y_center + 1, pencil);
}

void ObjectController::drawMap()
{
  int y = this->size().height();
  int x = this->size().width();
  int x_map_pix = map.x_meters * pix_per_meter;
  int y_map_pix = map.y_meters * pix_per_meter;
  int coor_0_x = (x_center) - (x_map_pix / 2);
  int coor_0_y = (y_center) - (y_map_pix / 2);
  scene->addRect(coor_0_x, coor_0_y, x_map_pix, y_map_pix, pencil, brush);
}

void ObjectController::drawCoordinates(int x, int y)
{
  if (map.set)
  {
    double* ptr;
    double res[2];
    ptr = res;
    toMapCoor(x, y, ptr);

    std::string str = boost::str(boost::format("x: %.3d y: %.3d") % ptr[0] % ptr[1]);
    coor_x = ptr[0];
    coor_y = ptr[1];
    coordinates->setPlainText(QString::fromUtf8(str.c_str()));
    coordinates->adjustSize();
    coordinates->setPos(this->size().width() - coordinates->document()->size().rwidth(),
                        this->size().height() - coordinates->document()->size().rheight());
  }
}

void ObjectController::drawObjects()
{
  for (int i = 0; i < objects.drones.size(); i++)
  {
    drawDrone(i);
  }
  
  for (int i = 0; i < objects.boxes.size(); i++)
  {
    drawBox(i);
  }

  for (int i = 0; i < objects.cylinders.size(); i++)
  {
    drawCylinder(i);
  }

  for (int i = 0; i < objects.obstacles.size(); i++)
  {
    drawObstacle(i);
  }
  drawPoints();
  drawTrajectories();
}

void ObjectController::drawDrone(int i)
{
  Drone drone = objects.drones[i];
  QPixmap p(":/images/images/drone-icon.png");
  QGraphicsPixmapItem* item = new QGraphicsPixmapItem(p.scaled(pix_per_meter * drone.x_size * default_real_drone_percent,
                                                               pix_per_meter * drone.y_size * default_real_drone_percent));
  
  double drone_center_x = (x_center + drone.x_coor * pix_per_meter / 2) ;
  double drone_center_y = (y_center - drone.y_coor * pix_per_meter / 2) ;
  double drone_size_x = drone.x_size * pix_per_meter * default_real_drone_percent / 2;
  double drone_size_y = drone.y_size * pix_per_meter * default_real_drone_percent / 2;
  
  item->setPos(drone_center_x - drone_size_x, drone_center_y - drone_size_y);
  item->setTransformOriginPoint(drone_size_x, drone_size_y);
  item->setRotation(-drone.degrees);
  scene->addItem(item);

  // toSceneCoor(drone.x_coor, drone.y_coor, real_coor); 
  QGraphicsLineItem* arrow = new QGraphicsLineItem(drone_center_x, drone_center_y, drone_center_x, drone_center_y + drone_size_y * 2);
  arrow->setTransformOriginPoint(drone_center_x, drone_center_y);
  arrow->setRotation(-drone.degrees);
  scene->addItem(arrow);
  
  // Draw arrow
  /* if (drone.selected)
  {
    item->setOpacity(0.5);
    arrow->setOpacity(0.2);
  }
  drone.item = item;
  drone.arrow = arrow;
  objects.drones[i] = drone; */
}

void ObjectController::drawBox(int i)
{
  pencil.setColor(QColor::fromRgb(94, 44, 0));
  brush.setStyle(Qt::Dense6Pattern);
  brush.setColor(QColor::fromRgb(94, 44, 0));
  
  Box box = objects.boxes[i];
  QGraphicsRectItem* item = scene->addRect(0, 0, box.x_size * pix_per_meter, box.y_size * pix_per_meter, pencil, brush);
  pencil.setColor(Qt::black);
  brush.setStyle(Qt::NoBrush);
  brush.setColor(Qt::black);
  item->setPos((x_center + box.x_coor * pix_per_meter / 2) - box.x_size * pix_per_meter / 2, 
              (y_center - box.y_coor * pix_per_meter / 2) - box.y_size * pix_per_meter / 2);
  item->setTransformOriginPoint(box.x_size * pix_per_meter / 2 , box.y_size * pix_per_meter / 2 );
  item->setRotation(-box.degrees * 180 / PI);
  

  /* if (box.selected)
  {
    item->setOpacity(0.3);
  }
  box.item = item;
  objects.boxes[i] = box; */
}

void ObjectController::drawCylinder(int i)
{
  pencil.setColor(QColor::fromRgb(94, 44, 0));
  brush.setStyle(Qt::Dense6Pattern);
  brush.setColor(QColor::fromRgb(94, 44, 0));
  
  Cylinder cylinder = objects.cylinders[i];
  QGraphicsEllipseItem* item = scene->addEllipse(0, 0, cylinder.radius * pix_per_meter, cylinder.radius * pix_per_meter, pencil, brush);
  item->setPos((x_center + cylinder.x_coor * pix_per_meter / 2) - cylinder.radius * pix_per_meter / 2, 
               (y_center - cylinder.y_coor * pix_per_meter / 2) - cylinder.radius * pix_per_meter / 2);
  pencil.setColor(Qt::black);
  brush.setStyle(Qt::NoBrush);
  brush.setColor(Qt::black);
  
  /* if (cylinder.selected)
  {
    item->setOpacity(0.3);
  }
  cylinder.item = item;
  objects.cylinders[i] = cylinder; */
}

void ObjectController::drawObstacle(int i)
{
  Obstacle obstacle = objects.obstacles[i];
  QPixmap p(":/images/images/pole.png");
  QGraphicsPixmapItem* item = new QGraphicsPixmapItem(p.scaled(pix_per_meter * obstacle.radius * default_real_drone_percent, 
                                                               pix_per_meter * obstacle.radius * default_real_drone_percent));
  
  double obstacle_center_x = (x_center + obstacle.x_coor * pix_per_meter / 2);
  double obstacle_center_y = (y_center - obstacle.y_coor * pix_per_meter / 2);
  double obstacle_size_x = obstacle.radius * pix_per_meter * default_real_drone_percent / 2;
  double obstacle_size_y = obstacle.radius * pix_per_meter * default_real_drone_percent / 2;
  
  item->setPos(obstacle_center_x - obstacle_size_x, obstacle_center_y - obstacle_size_y);
  scene->addItem(item);
  
  /* if (obstacle.selected)
  {
    item->setOpacity(0.5);
  }
  obstacle.item = item;
  objects.obstacles[i] = obstacle; */
} 

void ObjectController::drawPoints()
{
  if (objects.drones.size() > 0)
  {
    for (int j = 0; j < objects.drones.size(); j++)
    {
      if (objects.drones[j].points.size() > 0)
      {
        for (int i = 0; i < objects.drones.at(j).points.size() - 1; i++)
        {
          int coor[2];
          int coor1[2];
          int* res;
          int* res1;
          res = coor;
          res1 = coor1;
          toSceneCoor(objects.drones[j].points[i].x, objects.drones[j].points[i].y, res);
          toSceneCoor(objects.drones[j].points[i + 1].x, objects.drones[j].points[i + 1].y, res1);
          scene->addLine(res[0], res[1], res1[0], res1[1], pencil);
        }
      }
    }
  }
}

void ObjectController::drawTrajectories()
{
  pencil.setColor(Qt::red);
  if (objects.trajectories.size() > 0)
  {
    for (int i = 0; i < objects.trajectories.size(); i++)
    {
      int j;
      for (j = 0; j < objects.trajectories[i].size() - 1; j++)
      {
        int coor[2];
        int coor1[2];
        int* res;
        int* res1;
        res = coor;
        res1 = coor1;
        toSceneCoor(objects.trajectories.at(i).at(j).x, objects.trajectories.at(i).at(j).y, res);
        toSceneCoor(objects.trajectories.at(i).at(j + 1).x, objects.trajectories.at(i).at(j + 1).y, res1);
        scene->addLine(res[0], res[1], res1[0], res1[1], pencil);
      }
      int coor[2];
      int* res;
      res = coor;
      toSceneCoor(objects.trajectories.at(i).at(j).x, objects.trajectories.at(i).at(j).y, res);
      scene->addEllipse(res[0] - 5, res[1] - 5, 10, 10, pencil);
    }
  }
  pencil.setColor(Qt::black);
}

void ObjectController::updateDrone(ObjectController::Drone drone, int drone_pos)
{
  Drone d;
  d = objects.drones[drone_pos];
  //d.id = drone.id;
  //d.x_size = drone.x_size;
  //d.y_size = drone.y_size;
  d.x_coor = drone.x_coor;
  d.y_coor = drone.y_coor;
  d.degrees = drone.degrees;
  //d.points = drone.points;
  //d.take_off = drone.take_off;
  objects.drones[drone_pos] = d;
  reDraw();
  //drawDrone(drone_pos);
}

void ObjectController::updateObstacles(ObjectController::Obstacle obstacle)
{
  for(std::vector<int>::size_type i = 0; i != objects.obstacles.size(); i++) 
  {
    if (objects.obstacles[i].name.compare(obstacle.name) == 0)
    {
      already_exist = true;
      objects.obstacles[i] = obstacle;
      reDraw();
      break;
    }
  }
  if(!already_exist)
  {
    QPixmap p(":/images/images/pole.png");
    QGraphicsPixmapItem* item = new QGraphicsPixmapItem(p.scaled(pix_per_meter * obstacle.radius * default_real_drone_percent, 
                                                                pix_per_meter * obstacle.radius * default_real_drone_percent));
    
    double obstacle_center_x = (x_center + obstacle.x_coor * pix_per_meter / 2);
    double obstacle_center_y = (y_center - obstacle.y_coor * pix_per_meter / 2);
    double obstacle_size_x = obstacle.radius * pix_per_meter * default_real_drone_percent / 2;
    double obstacle_size_y = obstacle.radius * pix_per_meter * default_real_drone_percent / 2;
    
    item->setPos(obstacle_center_x - obstacle_size_x, obstacle_center_y - obstacle_size_y);
    scene->addItem(item);
    
    obstacle.item = item;
    objects.obstacles.push_back(obstacle); 
  }
  already_exist = false;
}

void ObjectController::updateTrajectory(std::vector<ObjectController::Point> trajectory)
{
  objects.trajectories.push_back(trajectory);
  pencil.setColor(Qt::red);
  int j;
  for (j = 0; j < trajectory.size() - 1; j++)
  {
    int coor[2];
    int coor1[2];
    int* res;
    int* res1;
    res = coor;
    res1 = coor1;
    toSceneCoor(trajectory.at(j).x, trajectory.at(j).y, res);
    toSceneCoor(trajectory.at(j + 1).x, trajectory.at(j + 1).y, res1);
    scene->addLine(res[0], res[1], res1[0], res1[1], pencil);
  }
  int coor[2];
  int* res;
  res = coor;
  toSceneCoor(trajectory.at(j).x, trajectory.at(j).y, res);
  scene->addEllipse(res[0] - 5, res[1] - 5, 10, 10, pencil);
  pencil.setColor(Qt::black);
}

void ObjectController::toMapCoor(int x, int y, double* result)
{
  //double size_y = this->size().height();
  //double size_x = this->size().width();
  //double x_map_pix = map.x_meters * pix_per_meter;
  //double y_map_pix = map.y_meters * pix_per_meter;
  double real_coor_y = (-y  + y_center) / pix_per_meter * 2;
  double real_coor_x = (x - x_center) / pix_per_meter * 2;
  result[0] = real_coor_x;
  result[1] = real_coor_y;
}

void ObjectController::toSceneCoor(double x, double y, int* result)
{
  //double size_y = this->size().height();
  //double size_x = this->size().width();
  //double x_map_pix = map.x_meters * pix_per_meter;
  //double y_map_pix = map.y_meters * pix_per_meter;
  //double coor_0_x = (x_center) - (x_map_pix / 2) - (map.x_init * pix_per_meter);
  //double coor_0_y = (y_center) - (y_map_pix / 2) + (map.y_init * pix_per_meter);
  result[0] = (x_center + x * pix_per_meter / 2);
  result[1] = (y_center - y * pix_per_meter / 2);
  //result[0] = x * pix_per_meter + coor_0_x;
  //result[1] = -(y * pix_per_meter - coor_0_y - y_map_pix);
}

void ObjectController::reDraw()
{
  if (map.set)
  {
    scene->clear();
    coordinates = new QGraphicsTextItem("");
    scene->addItem(coordinates);
    sticky_ellipse = scene->addEllipse(-5, -5, 5, 5, pencil, brush);
    drawGuide();
    drawMap();
    drawObjects();
  }
}

void ObjectController::clearTrajectories()
{
  for (int i = 0; i < objects.drones.size(); i++)
  {
    objects.drones[i].points.clear();
  }
  objects.trajectories.clear();
  reDraw();
}

void ObjectController::resizeEvent(QResizeEvent* event)
{
  x_center = this->width() / 2;
  y_center = this->height() / 2;
  setAutoPixPerMeter();
  reDraw();
}

void ObjectController::wheelEvent(QWheelEvent* event)
{
  int* ptr;
  int res[2];
  ptr = res;
  toSceneCoor(coor_x, coor_y, ptr);
  if (event->delta() < 0 && pix_per_meter - (pix_per_meter / 4) > 0)
  {
    pix_per_meter -= pix_per_meter / 4;
  }
  else if (event->delta() > 0)
  {
    pix_per_meter += pix_per_meter / 3;
  }
  int* ptr2;
  int res2[2];
  ptr2 = res2;
  toSceneCoor(coor_x, coor_y, ptr2);
  ptr2[0] = ptr[0] - ptr2[0];
  ptr2[1] = ptr[1] - ptr2[1];
  x_center = x_center + ptr2[0];
  y_center = y_center + ptr2[1];
  reDraw();
  event->accept();
}

void ObjectController::mouseMoveEvent(QMouseEvent* event)
{
  drawCoordinates(event->x(), event->y());
  if (map.set)
  {
    if (drag)
    {
      int* ptr;
      int res[2];
      ptr = res;
      toSceneCoor(coor_x, coor_y, ptr);
      double x_desp = ptr[0] - x_drag;
      double y_desp = ptr[1] - y_drag;
      x_drag = ptr[0];
      y_drag = ptr[1];
      x_center = x_center + x_desp;
      y_center = y_center + y_desp;
      reDraw();
    }
  }
}

void ObjectController::mousePressEvent(QMouseEvent* event)
{
  if (map.set)
  {
    if (event->button() != Qt::MiddleButton)
    {
      drag = true;
      QApplication::setOverrideCursor(Qt::ClosedHandCursor);
      int* ptr;
      int res[2];
      ptr = res;
      toSceneCoor(coor_x, coor_y, ptr);
      x_drag = ptr[0];
      y_drag = ptr[1];
    }
    else
    {
      x_center = this->width() / 2;
      y_center = this->height() / 2;
      setAutoPixPerMeter();
      reDraw();
    }
  }
}

void ObjectController::mouseReleaseEvent(QMouseEvent* event)
{
  if (map.set)
  {
    drag = false;
    QApplication::setOverrideCursor(Qt::OpenHandCursor);
    reDraw();
  }
}

void ObjectController::addNewDrone()
{
  Drone drone;
  drone.x_size = default_drone_x_size;
  drone.y_size = default_drone_y_size;
  drone.x_coor = coor_x;
  drone.y_coor = coor_y;
  drone.degrees = default_drone_degrees;
  drone.take_off = default_take_off_value;
  drone.id = default_drone_id;
  default_drone_id++;
  objects.drones.push_back(drone);
  reDraw();
}

/* void ObjectController::addNewWall(std::string description, double x_size, double y_size, double x_coor, double y_coor)
{
  Wall wall;
  wall.description = description;
  wall.x_size = x_size;
  wall.y_size = y_size;
  if (x_coor == 0 && y_coor == 0)
  {
    wall.x_coor = coor_x;
    wall.y_coor = coor_y;
  }
  else
  {
    wall.x_coor = x_coor;
    wall.y_coor = y_coor;
  }
  wall.degrees = 0;
  wall.id = default_wall_id;
  default_wall_id++;
  objects.walls.push_back(wall);
  reDraw();
} */

