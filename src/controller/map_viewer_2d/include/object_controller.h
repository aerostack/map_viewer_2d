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

#ifndef OBJECT_CONTROLLER_H
#define OBJECT_CONTROLLER_H

#include <QWidget>
#include <QGraphicsView>
#include <QObject>
#include <QGridLayout>
#include <QWheelEvent>
#include <QGraphicsTextItem>
#include <QApplication>
#include <QString>
#include <QGraphicsEllipseItem>
#include <QGraphicsPixmapItem>
#include <QGraphicsSceneDragDropEvent>
#include <QSizeF>
#include <QTextDocument>

#include <boost/format.hpp>
#include <cmath>
#include <time.h>
#include <iostream>

#define PI 3.14159265

class ObjectController : public QGraphicsView
{
  Q_OBJECT

public:
  explicit ObjectController(QWidget* parent = 0);
  ~ObjectController();
  
  struct Point
  {
    double x;
    double y;
  };

  struct Map
  {
    bool set = false;
    double x_meters;
    double y_meters;
    double x_init = 0;
    double y_init = 0;
  } map;

  struct Drone
  {
    int id;
    double x_size;
    double y_size;
    double x_coor;
    double y_coor;
    double degrees;
    double take_off;
    bool selected = false;
    QGraphicsPixmapItem* item;
    QGraphicsLineItem* arrow;
    std::vector<Point> points;
  };

  struct Obstacle
  {
    int id;
    std::string name;
    double radius;
    double x_coor;
    double y_coor;
    QGraphicsPixmapItem* item;
  };

  struct Box
  {
    std::string name;
    double x_size;
    double y_size;
    double z_size;
    double x_coor;
    double y_coor;
    double degrees;   // yaw
  
    QGraphicsRectItem* item;
    //bool selected = false;
  };

  struct Cylinder
  {
    std::string name;
    double radius;
    double length;
    double x_coor;
    double y_coor;

    QGraphicsEllipseItem* item;
    //bool selected = false;
  };

  struct Objects
  {
    std::vector<Drone> drones;
    std::vector<Box> boxes;
    std::vector<Cylinder> cylinders;
    std::vector<Obstacle> obstacles;
    std::vector<std::vector<Point>> trajectories;
    Map map;
  };

   /*!********************************************************************************************************************
   *  \brief      This method sets the map properties
   **********************************************************************************************************************/
  void setMap(double x_meters, double y_meters, double x_init = 0, double y_init = 0);
  
  /*!********************************************************************************************************************
   *  \brief      This method checks if void setMap has been called
   *  \return     True or false
   **********************************************************************************************************************/
  bool isMapSet();
  
  /*!********************************************************************************************************************
   *  \brief      This method clear the trajectories 
   **********************************************************************************************************************/
  void clearTrajectories();

private:
  int x_center;
  int y_center;
  int x_drag;
  int y_drag;
  int pix_guide;
  bool drag = false;
  bool click = false;
  double pix_per_meter = 100;
  clock_t time;

  Objects objects;
  Objects previous_status;

  // Drone default values
  int default_drone_id = 1;
  double default_drone_x_size = 0.6;
  double default_drone_y_size = 0.6;
  double default_drone_degrees = 90;  //+Y orientation
  double default_take_off_value = 0.7;
  double default_real_drone_percent = 0.7;
  
  QGraphicsScene* scene;
  QGraphicsTextItem* coordinates;
  QPen pencil;
  QBrush brush;
  QGraphicsEllipseItem* sticky_ellipse;

  bool delete_on_unselect = false;
  bool mission_mode = false;
  bool already_exist = false;

  double coor_x;
  double coor_y;

  /*!********************************************************************************************************************
   *  \brief      This method refreshes the graphical representation of the map by erasing it and drawing it.
   **********************************************************************************************************************/
  void reDraw();

  /*!********************************************************************************************************************
   *  \brief      This method draws the map grids
   **********************************************************************************************************************/
  void drawGuide();

  /*!********************************************************************************************************************
   *  \brief      This method draws the outer limits of the map
   **********************************************************************************************************************/
  void drawMap();

  /*!********************************************************************************************************************
   *  \brief      This method makes calls to all the methods that draw the different types of objects.
   **********************************************************************************************************************/
  void drawObjects();

  /*!********************************************************************************************************************
   *  \brief      This method draws the drone or drones on the map
   **********************************************************************************************************************/
  void drawDrone(int i);

  /*!********************************************************************************************************************
   *  \brief      This method draws the boxes on the map
   **********************************************************************************************************************/
  void drawBox(int i);

  /*!********************************************************************************************************************
   *  \brief      This method draws the cylinders on the map
   **********************************************************************************************************************/
  void drawCylinder(int i);

  /*!********************************************************************************************************************
   *  \brief      This method draws the obstacles on the map
   **********************************************************************************************************************/
  void drawObstacle(int i);

  /*!********************************************************************************************************************
   *  \brief      This method draws the points
   **********************************************************************************************************************/
  void drawPoints();

  /*!********************************************************************************************************************
   *  \brief      This method draws the trajectories that represent the drone's moves.
   **********************************************************************************************************************/
  void drawTrajectories();

  /*!********************************************************************************************************************
   *  \brief      This method sets the number of pixels that correspond with one meter
   **********************************************************************************************************************/
  void setAutoPixPerMeter();

  /*!********************************************************************************************************************
   *  \brief      This method is used to set the initial four walls that delimitate the boundaries of the map
   **********************************************************************************************************************/
  void setAutoMapWalls();

  /*!********************************************************************************************************************
   *  \brief      This method returns in result a vector with the real coordinates that correspond with the given
   *              coordinates.
   **********************************************************************************************************************/
  void toMapCoor(int x, int y, double* result);

  /*!********************************************************************************************************************
   *  \brief      This method returns in result a vector with the virtual coordinates that correspond with the given
   *              coordinates.
   **********************************************************************************************************************/
  void toSceneCoor(double x, double y, int* result);

  /*!********************************************************************************************************************
   *  \brief      This is a overwritten Qt event that will force the map to refresh whenever a QResizeEvent happens.
   **********************************************************************************************************************/
  void resizeEvent(QResizeEvent* event);

  /*!********************************************************************************************************************
   *  \brief      This is an overwritten Qt event that allows the user to zoom in or out with the mouse wheel.
   **********************************************************************************************************************/
  void wheelEvent(QWheelEvent* event);

  /*!********************************************************************************************************************
   *  \brief      This method draws the coordinates and places them in the correct position
   **********************************************************************************************************************/
  void drawCoordinates(int x, int y);

  /*!********************************************************************************************************************
   *  \brief      This is an overwritten Qt event that allows the user to move the map by dragging it, or move an object
   *              by dragging it aswell
   **********************************************************************************************************************/
  void mouseMoveEvent(QMouseEvent* event);

  /*!********************************************************************************************************************
   *  \brief      This is an overwritten Qt event that links the mouseclick with adding to the map the previously
   *              selected object
   **********************************************************************************************************************/
  void mousePressEvent(QMouseEvent* event);

  /*!********************************************************************************************************************
   *  \brief      This is an overwritten Qt event that forces the map to be refreshed whenever the user releases the
   *              mouse button
   **********************************************************************************************************************/
  void mouseReleaseEvent(QMouseEvent* event);

  /*!********************************************************************************************************************
   *  \brief      This method adds a new drone to the drone vector
   **********************************************************************************************************************/
  void addNewDrone();

  /*!********************************************************************************************************************
   *  \brief      This method adds a new wall to the walls vector
   **********************************************************************************************************************/
  //void addNewWall(std::string description = "wall", double x_size = 0.5, double y_size = 0.5, double x_coor = 0,
                  //double y_coor = 0);

public:
  /*!********************************************************************************************************************
   *  \brief      This method returns the list of current objects.
   **********************************************************************************************************************/
  ObjectController::Objects getObjects();

  /*!********************************************************************************************************************
   *  \brief      This method sets the list of objects to the given list and refreshes the map.
   **********************************************************************************************************************/
  void setObjects(Objects objects);

  /*!********************************************************************************************************************
   *  \brief      This method updates the drone's position
   **********************************************************************************************************************/
  void updateDrone(ObjectController::Drone drone, int drone_pos);

  /*!********************************************************************************************************************
   *  \brief      This method updates the drone's trajectory
   **********************************************************************************************************************/
  void updateTrajectory(std::vector<Point> trajectory);

  /*!********************************************************************************************************************
   *  \brief      This method updates the poles
   **********************************************************************************************************************/
  void updateObstacles(Obstacle obstacle);

Q_SIGNALS:
};

#endif // OBJECT_CONTROLLER_H
