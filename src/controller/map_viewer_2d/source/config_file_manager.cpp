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

#include "../include/config_file_manager.h"

ConfigFileManager::ConfigFileManager() : QObject()
{
  n.param<std::string>("robot_namespace", robot_namespace, "drone1");
  n.param<std::string>("my_stack_directory", my_stack_directory, "~/workspace/ros/aerostack_catkin_ws/src/aerostack_stack");
  n.param<std::string>("robot_config_path", configuration_folder, "~/workspace/ros/aerostack_catkin_ws/src/aerostack_stack/configs/drone1");
}

ConfigFileManager::~ConfigFileManager()
{
}

ObjectController::Objects ConfigFileManager::importObjects()
{
  drone_dir = configuration_folder;
 
  /* Import map attributes */
  filename = drone_dir + "/configFile.xml";  // configFile.xml
  document.load_file(filename.c_str());
  std::cout << filename.c_str() << std::endl;
  
  dimensions = document.child("map").child("config").child("dimensions");
  dimensions_vector = returnValuesPtr(dimensions.child_value());
  objects.map.set = true;
  objects.map.x_meters = dimensions_vector[0];
  objects.map.y_meters = dimensions_vector[1];
  
  initPoint = document.child("map").child("config").child("initPoint");
  initPoint_vector = returnValuesPtr(initPoint.child_value());
  objects.map.x_init = initPoint_vector[0];
  objects.map.y_init = initPoint_vector[1];

  /* Import drone attributes */
  robot = document.child("robot").child("dimensions");
  robot_vector = returnValuesPtr(robot.child_value());
  drone.id = std::stoi(robot_namespace.substr(5));
  drone.x_size = robot_vector[0];
  drone.y_size = robot_vector[1];
  
  filename = drone_dir + "/ekf_state_estimator_config.xml";  // ekf_state_estimator_config.xml
  document.load_file(filename.c_str());
  std::cout << filename.c_str() << std::endl;
  
  position = document.child("take_off_site").child("position");
  drone.x_coor = std::stod(position.child("x").child_value());
  drone.y_coor = std::stod(position.child("y").child_value());
  
  attitude = document.child("take_off_site").child("attitude");
  drone.degrees = std::stod(attitude.child("yaw").child_value());

  filename = drone_dir + "/params_localization_obs.xml";  // params_localization_obs.xml
  document.load_file(filename.c_str());
  std::cout << filename.c_str() << std::endl;
  
  z_0 = document.child("main").child("params").child("z_0");
  drone.take_off = std::stod(z_0.child_value());
  objects.drones.push_back(drone);

  /* Import shapes attributes */
  filename = drone_dir + "/geometry_map.world";  // geometry_map.world
  document.load_file(filename.c_str());
  std::cout << filename.c_str() << std::endl;
  world = document.child("sdf").child("world");
  state = world.child("state");
	
  for (pugi::xml_node model = world.child("model"); model; model = model.next_sibling())
  {
    pose = model.child("pose");
    if(pose!=NULL)
    {
      geometry = model.child("link").child("visual").child("geometry");
      shape = geometry.first_child().name();
			pose_vector = returnValuesPtr(pose.child_value());
			
      /* Import boxes attributes */
      if (shape == "box"){
				size_vector = returnValuesPtr(geometry.child("box").child("size").child_value());
				box.name = model.attribute("name").value();
				box.x_size = size_vector[0] / 2;
				box.y_size = size_vector[1] / 2;
				box.z_size = size_vector[2] / 2;
        for (pugi::xml_node state_model = state.child("model"); state_model; state_model = state_model.next_sibling())
        {
          std::string  name = state_model.attribute("name").value();
		      if (name.compare(box.name) == 0)
          {
            pose = state_model.child("pose");
            scale = state_model.child("scale");
			      pose_vector = returnValuesPtr(pose.child_value());
            scale_vector = returnValuesPtr(scale.child_value());
            box.x_coor = pose_vector[0];
				    box.y_coor = pose_vector[1];
				    box.degrees = pose_vector[5];
            if (scale_vector != ones_vec)
            {
              box.x_size = 	scale_vector[0] / 2;
              box.y_size = 	scale_vector[1] / 2;
              box.z_size = 	scale_vector[2] / 2;
              break;
            }
          }
        }
        objects.boxes.push_back(box);
			}
			
      /* Import cylinders attributes */
      else if (shape == "cylinder")
			{
				cylinder.name = model.attribute("name").value();
				cylinder.radius = std::stod(geometry.child("cylinder").child("radius").child_value());
				cylinder.length = std::stod(geometry.child("cylinder").child("length").child_value());
        for (pugi::xml_node state_model = state.child("model"); state_model; state_model = state_model.next_sibling())
        {
          std::string  name = state_model.attribute("name").value();
		      if(name.compare(cylinder.name) == 0)
          {
            pose = state_model.child("pose");
            scale = state_model.child("scale");
			      pose_vector = returnValuesPtr(pose.child_value());
            scale_vector = returnValuesPtr(scale.child_value());
            cylinder.x_coor = pose_vector[0];
				    cylinder.y_coor = pose_vector[1];
            if (scale_vector != ones_vec)
            {
              cylinder.radius = scale_vector[0] / 2;
              cylinder.length = scale_vector[2];
              break;
            }
          }
        }
        objects.cylinders.push_back(cylinder);
			}
			else
			{
				std::cout << "Only accepted box and cylinder geometry shapes" << "\n";
			}
    }
  }
  return objects;
}

std::vector<double> ConfigFileManager::returnValuesPtr(std::string value)
{
  std::vector<double> result;
  char* copy;
  char* token;
  copy = strdup(value.c_str());
  token = strtok(copy, " \t");
  std::string str;
  while (token != NULL)
  {
    str = boost::str(boost::format("%.4d") % token);
    result.push_back(std::stod(str));
    token = strtok(NULL, " \t");
  }
  return result;
}