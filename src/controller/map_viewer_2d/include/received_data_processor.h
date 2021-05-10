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

#ifndef RECEIVED_DATA_H
#define RECEIVED_DATA_H

#include <ros/ros.h>
#include <Eigen/Geometry>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <QObject>

#include <angles/angles.h>
#include <tf/tf.h>
#include <iostream>

#include "odometry_state_receiver.h"
#include "obstacles_receiver.h"
#include "poses_receiver.h"
#include "object_controller.h"

class MapViewer2DConnection;

class ReceivedDataProcessor : public QObject
{
  Q_OBJECT

public:
  explicit ReceivedDataProcessor(ObjectController* object_controller = 0, 
                                 OdometryStateReceiver* odometry_receiver = 0,
                                 ObstaclesReceiver* obstacles_receiver = 0,
                                 PosesReceiver* poses_receiver = 0);
  ~ReceivedDataProcessor();
  
   /*!********************************************************************************************************************
   *  \brief      This method transforms roll pitch and yaw to Euler angle
   **********************************************************************************************************************/
  void toEulerAngle(const Eigen::Quaternionf& q, float& roll, float& pitch, float& yaw);

private:
  OdometryStateReceiver* odometry_receiver;
  ObstaclesReceiver* obstacles_receiver;
  PosesReceiver* poses_receiver;
  ObjectController* object_controller;

  std::string drone_id_namespace;
  ros::NodeHandle n;
  double precision = 0.05;
  float roll, pitch, yaw;

public Q_SLOTS:

  /*!********************************************************************************************************************
   *  \brief      This slot is executed when new drone's data is received
   **********************************************************************************************************************/
  void updateDrone();

  /*!********************************************************************************************************************
   *  \brief      This slot is executed when new obstacle's data is received
   **********************************************************************************************************************/
  void updateObstacles();

  /*!********************************************************************************************************************
   *  \brief      This slot is executed when new trajectories' data is received
   **********************************************************************************************************************/
  void updateTrajectory();

  /*!********************************************************************************************************************
   *  \brief      This slot updates the trajectories
   **********************************************************************************************************************/
  //void updateDronesTrajectories();

  /*!************************************************************************
    *  \brief  Connects the different topic receiver objects to the ReceivedDataProcessor object
  ***************************************************************************/
  void connectReceiversToReceivedDataProcessor(QObject* odometry_receiver, QObject* obstacles_receiver, QObject* poses_receiver);
};

#endif  // RECEIVED_DATA_H
