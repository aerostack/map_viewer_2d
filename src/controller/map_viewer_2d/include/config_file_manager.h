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

#ifndef CONFIG_FILE_MANAGER_H
#define CONFIG_FILE_MANAGER_H

#include <ros/ros.h>

#include <iostream>
#include <pugixml.hpp>

#include <QObject>
#include <QMessageBox>

#include "object_controller.h"
#include "droneMsgsROS/configurationFolder.h"

class ConfigFileManager : public QObject
{
  Q_OBJECT

public:
  explicit ConfigFileManager();
  ~ConfigFileManager();

  /*!********************************************************************************************************************
   *  \brief      This method imports all the objects and map characteristics to this process' variables.
   **********************************************************************************************************************/
  ObjectController::Objects importObjects();

private:
  ros::NodeHandle n;
  ObjectController::Objects objects;

  std::string my_stack_directory;
  std::string robot_namespace;
  std::string drone_dir;
  std::string filename;
  std::string configuration_folder;

  //ObjectController::Objects objects;
  ObjectController::Drone drone;
  ObjectController::Box box;
  ObjectController::Cylinder cylinder;
  pugi::xml_document document;
  pugi::xml_node dimensions;
	pugi::xml_node initPoint;
	pugi::xml_node robot;
  pugi::xml_node world;
  pugi::xml_node state;
  pugi::xml_node scale;
	pugi::xml_node pose;
	pugi::xml_node geometry;
  pugi::xml_node position;
  pugi::xml_node attitude;
  pugi::xml_node z_0;
  std::vector<double> dimensions_vector;
	std::vector<double> initPoint_vector;
	std::vector<double> robot_vector;
	std::vector<double> pose_vector;
  std::vector<double> scale_vector;
	std::vector<double> size_vector;
  std::vector<double> ones_vec { 1, 1, 1};
  std::string shape;

  /*!********************************************************************************************************************
   *  \brief      This method translates a string to a vector of double type digits
   **********************************************************************************************************************/
  std::vector<double> returnValuesPtr(std::string value);
};
#endif // CONFIG_FILE_MANAGER_H
