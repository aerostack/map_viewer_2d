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

#ifndef MAP_VIEWER_2D_WIDGET_H
#define MAP_VIEWER_2D_WIDGET_H

#include <ros/ros.h>

//#include "aerostack_msgs/WindowEvent.h"

#include <QWidget>
#include <QGridLayout>
#include <QGraphicsView>
#include <QSignalMapper>
#include <QFileDialog>
#include <QMessageBox>
#include <QSplitter>
#include <QLabel>
#include <QPushButton>
#include <QRect>
#include <QGuiApplication>
#include <QScreen>
#include <QProcess>

#include "object_controller.h"
#include "odometry_state_receiver.h"
#include "obstacles_receiver.h"
#include "received_data_processor.h"
#include "config_file_manager.h"

#include "ui_map_viewer_2d_widget.h"
#include <boost/property_tree/ptree.hpp>
//#include <boost/property_tree/json_parser.hpp>
//#include "fstream"

class MapViewer2DConnection;
class ReceivedDataProcessor;

namespace Ui
{
  class MapViewer2DWidget;
}

class MapViewer2DWidget : public QWidget
{
  Q_OBJECT

public:
  explicit MapViewer2DWidget(int argc, char** argv, QWidget* parent = 0);
  ~MapViewer2DWidget();

  boost::property_tree::ptree root;

  ros::NodeHandle n;
  ros::ServiceClient configuration_folder_client;
  droneMsgsROS::configurationFolder::Request req;
  droneMsgsROS::configurationFolder::Response res;

  std::string robot_namespace;
  std::string initiate_behaviors;
  std::string configuration_folder;
  std::string prefixed_layout;
  std::string configuration_folder_service;
  std::string rosnamespace;
  std::string my_stack_workspace;

  ros::Publisher window_event_pub;
  ros::Subscriber window_event_sub;

  std::string window_event_topic;

  //aerostack_msgs::WindowEvent window_event_msg;


private:
  Ui::MapViewer2DWidget* ui;

  bool null = true;

  MapViewer2DConnection* map_viewer_2d_connection;
  ObjectController* object_controller;
  ConfigFileManager* config_file_manager;
  OdometryStateReceiver* odometry_receiver;
  ObstaclesReceiver* obstacle_receiver;
  PosesReceiver* poses_receiver;
  ReceivedDataProcessor* received_data_processor;

  QGridLayout* layout;
  QWidget* widget_config = NULL;
  QSignalMapper* mapper;
  QLabel* object_label;
  QPushButton* clear_button;

  /*!********************************************************************************************************************
   *  \brief   This method is the responsible for seting up connections.
   *********************************************************************************************************************/
  //void setUp();

  /*!************************************************************************
   *  \brief   Activated when a window is closed.
   ***************************************************************************/
  // void windowOpenCallback(const aerostack_msgs::WindowEvent& msg);

  /*!************************************************************************
   *  \brief    Kills the process
   ***************************************************************************/
  void killMe();

public Q_SLOTS:

  /*!********************************************************************************************************************
   *  \brief    This slot is executed when the MapViewer2DWidget is created or when the user wants to import the map
   *            from the Aerostack config files.
   **********************************************************************************************************************/
  void importMap();

  /*!********************************************************************************************************************
   *  \brief    This slot is executed when the user clicks the Clear button
   **********************************************************************************************************************/
  void clearTrajectories();

  /*!********************************************************************************************************************
   *  \brief    This method notifies main window that the widget was closed
   *********************************************************************************************************************/
  void closeEvent(QCloseEvent* event);

Q_SIGNALS:

  /*!********************************************************************************************************************
   *  \brief    This signal is emitted when the map edition is either canceled or accepted.
   **********************************************************************************************************************/
  void showing(QWidget*);
};
#endif  // MAP_VIEWER_2D_WIDGET_H
