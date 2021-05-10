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

#ifndef OBSTACLES_RECEIVER_H
#define OBSTACLES_RECEIVER_H

#include <ros/ros.h>

#include <QObject>

#include <aerostack_msgs/DynamicObstacles.h>

class ObstaclesReceiver : public QObject
{
  Q_OBJECT

public:
  ObstaclesReceiver();
  ~ObstaclesReceiver();

  /*!************************************************************************
   *  \brief    This method opens the subscriptions that this process needs.
   ***************************************************************************/
  void openSubscriptions(ros::NodeHandle n, std::string ros_namespace);

  /*!************************************************************************
   *  \brief    This method checks wether the subscriptions have been opened already
   *  \return   Returns true if the subscriptions have been successfully opened, false otherwise.
   ***************************************************************************/
  bool isReady();

  /*!************************************************************************
   *  \brief   This method returns the map's obstacles vector.
   ***************************************************************************/
  aerostack_msgs::DynamicObstacles::_obstacles_type getObstaclesVector();

private:
  bool subscriptions_complete;
  std::string obstacles_topic;
  ros::Subscriber obstacles_sub;
  aerostack_msgs::DynamicObstacles obstacles_msg;
  aerostack_msgs::DynamicObstacles::_obstacles_type obstacles_vector;

  /*!************************************************************************
   *  \brief    This method allows to refresh the obstacle's position within the map
   ***************************************************************************/
  void obstaclesPositionCallback(const aerostack_msgs::DynamicObstacles::ConstPtr& msg);

Q_SIGNALS:
  void updateMapInfo();
};
#endif  // OBSTACLES_RECEIVER_H
