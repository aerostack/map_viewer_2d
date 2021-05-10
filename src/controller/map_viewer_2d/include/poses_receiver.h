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

#ifndef POSES_RECEIVER_H
#define POSES_RECEIVER_H

#include <ros/ros.h>
#include <QObject>

#include <nav_msgs/Path.h>

class PosesReceiver : public QObject
{
  Q_OBJECT

public:
  PosesReceiver();
  ~PosesReceiver();

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
   *  \brief   This method returns the poses vector received by the message
   *  \return  Poses vector
   ***************************************************************************/
  nav_msgs::Path::_poses_type getPosesVector();

private:
  bool subscriptions_complete;
  std::string poses_topic;
  ros::Subscriber poses_sub;
  nav_msgs::Path poses_msg;
  nav_msgs::Path::_poses_type poses_vector;
  
  /*!***********************************************************************************************
   *  \brief   This method emit a SIGNAL when a nav_msgs::Path is received         
   **************************************************************************************************/
  void posesCallback(const nav_msgs::Path::ConstPtr& msg);

Q_SIGNALS:
  void sendTrajectory();
};

#endif  // POSES_RECEIVER_H
