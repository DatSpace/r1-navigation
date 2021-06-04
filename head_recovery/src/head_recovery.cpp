/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <head_recovery/head_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <math.h>

#define PI 3.14159265

// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(head_recovery::HeadRecovery, nav_core::RecoveryBehavior)

namespace head_recovery
{
  HeadRecovery::HeadRecovery() : initialized_(false)
  {
    movement_coords_[0][0] = 0;
    movement_coords_[0][1] = 15;

    movement_coords_[1][0] = 15;
    movement_coords_[1][1] = 0;

    movement_coords_[2][0] = 0;
    movement_coords_[2][1] = -15;

    movement_coords_[3][0] = -15;
    movement_coords_[3][1] = 0;
  }

  void HeadRecovery::initialize(std::string name, tf2_ros::Buffer *, costmap_2d::Costmap2DROS *, costmap_2d::Costmap2DROS *local_costmap)
  {
    if (!initialized_)
    {
      // get some parameters from the parameter server
      ros::NodeHandle private_nh("~/" + name);

      // Load a string array of the ports to connect and keep oepn
      std::vector<std::string> yarp_ports_default, yarp_ports;
      yarp_ports_default.push_back(std::string("/cer/head/rpc:i"));
      private_nh.param("yarp_ports", yarp_ports, yarp_ports_default);

      for (unsigned i = 0; i < yarp_ports.size(); i++)
      {
        yarp_ports_.push_back(yarp_ports[i]);
      }

      ros::NodeHandle n;
      pub_ = n.advertise<std_msgs::String>("yarp_rpc_publisher", 10);

      ros::Duration(0.5).sleep();

      std_msgs::String message;
      for (int i = 0; i < yarp_ports_.size(); i++)
      {
        message.data = "connect " + yarp_ports_[i];
        pub_.publish(message);
        ros::Duration(0.1).sleep();
      }

      initialized_ = true;
    }
    else
    {
      ROS_ERROR("You should not call initialize twice on this object, doing nothing");
    }
  }

  HeadRecovery::~HeadRecovery()
  {
  }

  void HeadRecovery::runBehavior()
  {
    if (!initialized_)
    {
      ROS_ERROR("This object must be initialized before runBehavior is called");
      return;
    }

    ROS_WARN("Head movement recovery behavior started.");

    std_msgs::String message;

    for (int i = 0; i <= 3; i++)
    {
      message.data = "write " + yarp_ports_[0] + " set pos 1 " + std::to_string(movement_coords_[i][0]);
      pub_.publish(message);
      message.data = "write " + yarp_ports_[0] + " set pos 0 " + std::to_string(movement_coords_[i][1]);
      pub_.publish(message);
      message.data = "write " + yarp_ports_[1] + " set pos 3 " + std::to_string(movement_coords_[i][0]);
      pub_.publish(message);
      message.data = "write " + yarp_ports_[1] + " set pos 2 " + std::to_string(movement_coords_[i][1]);
      pub_.publish(message);
      ros::Duration(2.0).sleep();
    }

    message.data = "write " + yarp_ports_[0] + " set pos 1 0";
    pub_.publish(message);
    message.data = "write " + yarp_ports_[0] + " set pos 0 0";
    pub_.publish(message);

    message.data = "write " + yarp_ports_[1] + " set pos 3 0";
    pub_.publish(message);
    message.data = "write " + yarp_ports_[1] + " set pos 2 0";
    pub_.publish(message);
  }
}; // namespace head_recovery
