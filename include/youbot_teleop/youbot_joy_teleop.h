/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Worcester Polytechnic Institute
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Worcester Polytechnic Institute nor the 
 *     names of its contributors may be used to endorse or promote 
 *     products derived from this software without specific prior 
 *     written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *   Author: Russell Toris
 *  Version: Aug 2, 2012
 *
 *********************************************************************/

/*!
 * \file youbot_joy_teleop.h
 * \brief Allows for control of the Kuka YouBot with a joystick.
 *
 * youbot_joy_teleop creates a ROS node that allows the control of a Kuka YouBot with a joystick.
 * This node listens to a /joy topic and sends messages to the /cmd_vel topic. Arm control is currently unimplemented.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date Aug 2, 2012
 */

#ifndef YOUBOT_JOY_TELEOP_H_
#define YOUBOT_JOY_TELEOP_H_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

/*!
 * \class youbot_joy_teleop
 * \brief Provides a bridge between the joy topic and the cmd_vel topic.
 *
 * The youbot_joy_teleop handles the translation between joystick commands and communication to the YouBot's /cmd_vel topic.
 */
class youbot_joy_teleop
{
public:
  /*!
   * \brief Creates a youbot_joy_teleop.
   *
   * Creates a youbot_joy_teleop object that can be used control the Kuka YouBot with a joystick. ROS nodes, services, and publishers are created and maintained within this object.
   */
  youbot_joy_teleop();

private:
  /*!
   * \brief joy topic callback function.
   *
   * \param joy the message for the joy topic
   */
  void joy_cback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::Publisher cmd_vel; /*!< the cmd_vel topic */
  ros::Subscriber joy_sub; /*!< the joy topic */
};

/*!
 * Creates and runs the youbot_joy_teleop node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif
