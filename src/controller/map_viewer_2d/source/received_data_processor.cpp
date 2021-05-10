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

#include "../include/received_data_processor.h"

ReceivedDataProcessor::ReceivedDataProcessor(ObjectController* object_controller,
                                             OdometryStateReceiver* odometry_receiver,
                                             ObstaclesReceiver* obstacles_receiver,
                                             PosesReceiver* poses_receiver) : QObject()
{
  this->odometry_receiver = odometry_receiver;
  this->obstacles_receiver = obstacles_receiver;
  this->poses_receiver = poses_receiver;
  this->object_controller = object_controller;

  n.param<std::string>("robot_namespace", drone_id_namespace, "drone1");

  this->connectReceiversToReceivedDataProcessor(this->odometry_receiver, 
                                                this->obstacles_receiver, 
                                                this->poses_receiver);
}

ReceivedDataProcessor::~ReceivedDataProcessor()
{
}

void ReceivedDataProcessor::connectReceiversToReceivedDataProcessor(QObject* odometry_receiver, 
                                                                    QObject* obstacles_receiver, 
                                                                    QObject* poses_receiver)
{
  QObject::connect(odometry_receiver, SIGNAL(updateDronePosition()), this, SLOT(updateDrone()));
  QObject::connect(obstacles_receiver, SIGNAL(updateMapInfo()), this, SLOT(updateObstacles()));
  QObject::connect(poses_receiver, SIGNAL(sendTrajectory()), this, SLOT(updateTrajectory()));
}

void ReceivedDataProcessor::updateDrone()
{
  geometry_msgs::PoseStamped::_pose_type msg = odometry_receiver->getPoseVector();
  ObjectController::Objects objects = object_controller->getObjects();

  // Vehicle orientation
  Eigen::Quaternionf q(msg.orientation.w,
                       msg.orientation.x,
                       msg.orientation.y,
                       msg.orientation.z);

  toEulerAngle(q, roll, pitch, yaw);

  ObjectController::Drone drone;
  drone = objects.drones[0];
  drone.x_coor = msg.position.x;
  drone.y_coor = msg.position.y;
  drone.degrees = angles::to_degrees(yaw);

  ObjectController::Point point;
  point.x = msg.position.x;
  point.y = msg.position.y;

  if (drone.points.size() > 0)
  {
    ObjectController::Point last_point = drone.points.at(drone.points.size() - 1);
    if (sqrt(pow((last_point.x - point.x), 2) + pow((last_point.y - point.y), 2)) > precision)
    {
      drone.points.push_back(point);
    }
  }
  else
  {
    if (point.x != 0 && point.y != 0)
      drone.points.push_back(point);
  }
  object_controller->updateDrone(drone, 0);
}

void ReceivedDataProcessor::updateObstacles()
{
  ObjectController::Objects objects = object_controller->getObjects();
  aerostack_msgs::DynamicObstacles::_obstacles_type obstacles_vector = obstacles_receiver->getObstaclesVector();
  for (int i = 0; i < obstacles_vector.size(); i++)
  {
    if (obstacles_vector[i].type == obstacles_vector[i].DRONE)
    {
      ObjectController::Obstacle obstacle;
      obstacle.id = i;
      obstacle.name = obstacles_vector[i].name;
      obstacle.radius = obstacles_vector[i].shape.dimensions[0];
      obstacle.x_coor = obstacles_vector[i].position.x;
      obstacle.y_coor = obstacles_vector[i].position.y;
      object_controller->updateObstacles(obstacle);
    }
  }
}

void ReceivedDataProcessor::updateTrajectory()
{
  nav_msgs::Path::_poses_type poses_vector = poses_receiver->getPosesVector();
  ObjectController::Objects objects = object_controller->getObjects();
  std::vector<std::vector<ObjectController::Point>> trajectories = objects.trajectories;
  std::vector<ObjectController::Point> points;
  for (int i = 0; i < poses_vector.size(); i++)
  {
    ObjectController::Point p;
    p.x = poses_vector[i].pose.position.x;
    p.y = poses_vector[i].pose.position.y;
    points.push_back(p);
  }
  if (points.size() > 0)
  {
    object_controller->updateTrajectory(points);
  }
}

/* void ReceivedDataProcessor::updateDronesTrajectories()
{
  droneMsgsROS::societyPose::_societyDrone_type drone_info_vector = society_receiver->getDronesPosition();
  ObjectController::Objects objects = object_controller->getObjects();
  for (int i = 0; i < drone_info_vector.size(); i++)
  {
    bool found = false;
    ObjectController::Point point;
    point.x = drone_info_vector[i].pose.x;
    point.y = drone_info_vector[i].pose.y;
    for (int j = 0; j < objects.drones.size(); j++)
    {
      if (drone_info_vector[i].id == objects.drones[j].id)
      {
        objects.drones[j].x_coor = drone_info_vector[i].pose.x;
        objects.drones[j].y_coor = drone_info_vector[i].pose.y;
        objects.drones[j].degrees = angles::to_degrees(drone_info_vector[i].pose.yaw);
        if (objects.drones[j].points.size() > 0)
        {
          ObjectController::Point last_point = objects.drones[j].points.at(objects.drones[j].points.size() - 1);
          if (sqrt(pow((last_point.x - point.x), 2) + pow((last_point.y - point.y), 2)) > precision)
          {
            objects.drones[j].points.push_back(point);
          }
        }
        object_controller->updateDrone(objects.drones[j], j);
        found = true;
      }
    }
    if (!found)
    {
      if (drone_info_vector[i].pose.x != 0 && drone_info_vector[i].pose.y != -10)
      {
        ObjectController::Drone drone;
        drone.id = drone_info_vector[i].id;
        drone.x_coor = drone_info_vector[i].pose.x;
        drone.y_coor = drone_info_vector[i].pose.y;
        drone.degrees = angles::to_degrees(drone_info_vector[i].pose.yaw);
        drone.x_size = objects.drones[0].x_size;
        drone.y_size = objects.drones[0].y_size;
        drone.points.push_back(point);
        object_controller->updateDrone(drone, -1);
      }
    }
  }
} */

void ReceivedDataProcessor::toEulerAngle(const Eigen::Quaternionf& q, float& roll, float& pitch, float& yaw)
{
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  yaw = atan2(siny_cosp, cosy_cosp);
}
