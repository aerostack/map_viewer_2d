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

#include <QApplication>

#include <ros/ros.h>

#include <signal.h>
#include <stdio.h>
#include <thread>

void signalhandler(int sig)
{
  if (sig == SIGINT)
  {
    qApp->quit();
  }
  else if (sig == SIGTERM)
  {
    qApp->quit();
  }
}

void spinnerThread()
{
  ros::spin();
}

int main(int argc, char* argv[])
{
  //ros::init(argc, argv, MODULE_NAME_MAP_VIEWER_2D);  // ros node started.
  ros::init(argc, argv, ros::this_node::getName());  // ros node started.

  QApplication app(argc, argv);
  MapViewer2DWidget widget(argc, argv);

  widget.show();
  std::thread thr(&spinnerThread);

  signal(SIGINT, signalhandler);
  signal(SIGTERM, signalhandler);

  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

  int result = app.exec();

  // ros::spin();
}
