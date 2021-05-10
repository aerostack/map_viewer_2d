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

#ifndef ODOMETRY_STATE_RECEIVER_H_
#define ODOMETRY_STATE_RECEIVER_H_

#include <ros/ros.h>
#include <ros/network.h>

#include <geometry_msgs/PoseStamped.h>

#include <QString>
#include <QThread>
#include <QtDebug>
#include <QStringListModel>

#include <string>
#include <sstream>

class OdometryStateReceiver : public QObject
{
  Q_OBJECT

public:
  OdometryStateReceiver();
  ~OdometryStateReceiver();

  /*!************************************************************************
   *  \brief   This method opens the subscriptions needed by the process.
   ***************************************************************************/
  void openSubscriptions(ros::NodeHandle n, std::string ros_namespace);
  
  /*!************************************************************************
   *  \brief   This method checks if the subscriptions are successfully opened
   *  \return  Returns true if they're succesfully opened, false otherwise
   ***************************************************************************/
  bool isReady();

  /*!************************************************************************
   *  \brief   This method returns the pose vector received by the message
   *  \return  Pose vector
   ***************************************************************************/
  geometry_msgs::PoseStamped::_pose_type getPoseVector();
  
private:
  bool subscriptions_complete;
  std::string geometry_pose_topic;
  ros::Subscriber geometric_pose_sub;
  geometry_msgs::PoseStamped drone_pose_msg;
  geometry_msgs::PoseStamped::_pose_type geometry_pose_vector;
  
  /*!***********************************************************************************************
   *  \brief   This method emit a SIGNAL when a geometry_msgs::PoseStamped is received         
   **************************************************************************************************/
  void droneEstimatedPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

Q_SIGNALS:
  void updateDronePosition();
};
#endif // ODOMETRY_STATE_RECEIVER_H_
