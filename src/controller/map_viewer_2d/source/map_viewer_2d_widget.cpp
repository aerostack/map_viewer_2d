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

#include "../include/map_viewer_2d_widget.h"

#include <QProcess>

#include <iostream>

MapViewer2DWidget::MapViewer2DWidget(int argc, char** argv, QWidget* parent): QWidget(parent), ui(new Ui::MapViewer2DWidget)
{
  Qt::WindowFlags flags = windowFlags();
  
  setWindowIcon(QIcon(":/images/images/map_viewer_2d.png"));
  setWindowTitle("Map Viewer 2D");
  //setWindowFlags(flags | Qt::WindowStaysOnTopHint);

  ros::start();
  ros::NodeHandle n;

  if (ros::this_node::getNamespace().compare("/") == 0)
    rosnamespace.append("/drone1");  // default namespace
  else
    rosnamespace.append(ros::this_node::getNamespace());

  odometry_receiver = new OdometryStateReceiver();
  obstacle_receiver = new ObstaclesReceiver();
  poses_receiver = new PosesReceiver();

  odometry_receiver->openSubscriptions(n,rosnamespace);
  obstacle_receiver->openSubscriptions(n,rosnamespace);
  poses_receiver->openSubscriptions(n,rosnamespace);

  std::locale::global(std::locale::classic());
  ui->setupUi(this);
  n.param<std::string>("my_stack_workspace", my_stack_workspace, "~/workspace/ros/aerostack_catkin_ws");
  n.param<std::string>("prefixed_layout", prefixed_layout, "normal");
  
  object_controller = new ObjectController(this);
  clear_button = new QPushButton("Clear");
  
  layout = ui->gridLayout;
  layout->addWidget(object_controller, 1, 0, 1, 7);
  layout->addWidget(clear_button, 3, 6, 1, 1);
  QObject::connect(clear_button, SIGNAL(clicked()), this, SLOT(clearTrajectories()));

  config_file_manager = new ConfigFileManager();
  received_data_processor = new ReceivedDataProcessor(object_controller, 
                                                      odometry_receiver, 
                                                      obstacle_receiver, 
                                                      poses_receiver);

  importMap();
  /* namespace pt = boost::property_tree;

  std::string layout_dir = std::getenv("AEROSTACK_STACK") + std::string("/stack/ground_control_system/"
                                                                        "graphical_user_interface/layouts/layout.json");

  pt::read_json(layout_dir, root);

  QScreen* screen = QGuiApplication::primaryScreen();
  QRect screenGeometry = screen->geometry();

  int y0 = screenGeometry.height() / 2;
  int x0 = screenGeometry.width() / 2;

  int height = root.get<int>("ENVIRONMENT_VIEWER.height");
  int width = root.get<int>("ENVIRONMENT_VIEWER.width");

  this->resize(width, height);
  this->move(x0 + root.get<int>("ENVIRONMENT_VIEWER.position." + prefixed_layout + ".x"),
             y0 + root.get<int>("ENVIRONMENT_VIEWER.position." + prefixed_layout + ".y")); */
  //setUp();
}

MapViewer2DWidget::~MapViewer2DWidget()
{
  delete ui;
  delete object_controller;
  delete layout;
  delete config_file_manager;
}

/* void EnvironmentWidget::setUp()
{
  n.param<std::string>("robot_namespace", robot_namespace, "drone1");
  n.param<std::string>("window_event_topic", window_event_topic, "window_event");

  // Subscribers
  window_event_sub = n.subscribe("/" + robot_namespace + "/" + window_event_topic, 10, &EnvironmentWidget::windowOpenCallback, this);

  // Publishers
  window_event_pub = n.advertise<aerostack_msgs::WindowEvent>("/" + robot_namespace + "/" + window_event_topic, 1, true);
} */

void MapViewer2DWidget::importMap()
{
  ObjectController::Objects objects;
  objects = config_file_manager->importObjects();
  object_controller->setObjects(objects);
}

void MapViewer2DWidget::clearTrajectories()
{
  object_controller->clearTrajectories();
}

void MapViewer2DWidget::closeEvent(QCloseEvent* event)
{
  //window_event_msg.window = aerostack_msgs::WindowEvent::MAP_VIEWER_2D;
  //window_event_msg.event = aerostack_msgs::WindowEvent::CLOSE;
  //window_event_pub.publish(window_event_msg);
}

void MapViewer2DWidget::killMe()
{
  qint64 pid = QCoreApplication::applicationPid();
  QProcess::startDetached("kill -9 " + QString::number(pid));
  
 /*  #ifdef Q_OS_WIN
    enum
    {
      ExitCode = 0
    };
    ::TerminateProcess(::GetCurrentProcess(), ExitCode);
  #else
    qint64 pid = QCoreApplication::applicationPid();
    QProcess::startDetached("kill -9 " + QString::number(pid));
  #endif  // Q_OS_WIN */
} 

/* void EnvironmentWidget::windowOpenCallback(const aerostack_msgs::WindowEvent& msg)
{
  window_event_msg = msg;

  if (window_event_msg.window == aerostack_msgs::WindowEvent::INTEGRATED_VIEWER && window_event_msg.event == aerostack_msgs::WindowEvent::MINIMIZE )
    showMinimized();

  namespace pt = boost::property_tree;

  std::string layout_dir = std::getenv("AEROSTACK_STACK") + std::string("/stack/ground_control_system/"
                                                                        "graphical_user_interface/layouts/layout.json");

  pt::read_json(layout_dir, root);

  QScreen* screen = QGuiApplication::primaryScreen();
  QRect screenGeometry = screen->geometry();

  int y0 = screenGeometry.height() / 2;
  int x0 = screenGeometry.width() / 2;
  int height = root.get<int>("ENVIRONMENT_VIEWER.height");
  int width = root.get<int>("ENVIRONMENT_VIEWER.width");

  if ( window_event_msg.window == aerostack_msgs::WindowEvent::TELEOPERATION_CONTROL ||
      window_event_msg.window == aerostack_msgs::WindowEvent::PYTHON_CONTROL ||
      window_event_msg.window == aerostack_msgs::WindowEvent::BEHAVIOR_TREE_INTERPRETER)
    this->move(x0 + root.get<int>("ENVIRONMENT_VIEWER.position.center.x"), y0 + root.get<int>("ENVIRONMENT_VIEWER.position.center.y"));

  if (window_event_msg.window == aerostack_msgs::WindowEvent::ALPHANUMERIC_INTERFACE_CONTROL)
    this->move(x0 + root.get<int>("ENVIRONMENT_VIEWER.position.right.x"), y0 + root.get<int>("ENVIRONMENT_VIEWER.position.right.y"));
}
 */