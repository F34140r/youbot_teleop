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
 * \file youbot_joy_teleop.cpp
 * \brief Allows for control of the Kuka YouBot with a joystick.
 *
 * youbot_joy_teleop creates a ROS node that allows the control of a Kuka YouBot with a joystick.
 * This node listens to a /joy topic and sends messages to the /cmd_vel topic. Arm control is currently unimplemented.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date Aug 2, 2012
 */

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <youbot_teleop/youbot_joy_teleop.h>

using namespace std;

youbot_joy_teleop::youbot_joy_teleop()
{
  // create the ROS topics
  cmd_vel = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  joy_sub = node.subscribe<sensor_msgs::Joy>("joy", 10, &youbot_joy_teleop::joy_cback, this);

  ROS_INFO("YouBot Joystick Teleop Started");
}

void youbot_joy_teleop::joy_cback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // create the twist message
  geometry_msgs::Twist twist;
  // left joystick controls the linear movement
  twist.linear.x = joy->axes.at(1);
  twist.linear.y = joy->axes.at(0);
  twist.linear.z = 0;
  // right joystick controls the angular movement
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = joy->axes.at(2);
  // send the twist command
  cmd_vel.publish(twist);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "youbot_joy_teleop");

  // initialize the joystick controller
  youbot_joy_teleop controller;

  // continue until a ctrl-c has occurred
  ros::spin();
}
