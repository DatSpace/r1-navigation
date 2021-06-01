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
#include <algorithm>
#include <math.h>

#define PI 3.14159265

// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(head_recovery::HeadRecovery, nav_core::RecoveryBehavior)

namespace head_recovery
{
  HeadRecovery::HeadRecovery(): initialized_(false), radius_(15), num_points_(100), yarp_head_port_("/cer/head/rpc:i")
  {
    for (int i = 0; i <= num_points_ - 1 ; i++){
      float x = cos(2 * PI / num_points_ * i) * radius_;
      float y = sin(2 * PI / num_points_ * i) * radius_;

      circle_points_[i][0] = x;
      circle_points_[i][1] = y;
    }

  }

  void HeadRecovery::initialize(std::string name, tf2_ros::Buffer*, costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
  {
    if (!initialized_)
    {
      // get some parameters from the parameter server
      ros::NodeHandle private_nh("~/" + name);

      // we'll simulate every degree by default
      //private_nh.param("sim_granularity", sim_granularity_, 0.017);

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

    ros::Rate r(20);
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<std_msgs::String>("yarp_rpc_publisher", 10);

    ros::Duration(0.1).sleep();

    std_msgs::String message;
    message.data = yarp_head_port_;
    vel_pub.publish(message);

    // while (n.ok())
    // {
      ros::Duration(0.1).sleep();
      message.data = "set vel 0 10";
      vel_pub.publish(message);
      message.data = "set vel 1 10";
      vel_pub.publish(message);

      for (int i = 0; i <= num_points_ - 1; i++){
        message.data = "set pos 1 " + std::to_string(circle_points_[i][0]);
        vel_pub.publish(message);
        ros::Duration(0.05).sleep();
        message.data = "set pos 0 " + std::to_string(circle_points_[i][1]);
        vel_pub.publish(message);
        ros::Duration(0.05).sleep();
      }

      message.data = "set pos 1 0";
      vel_pub.publish(message);
      ros::Duration(0.1).sleep();
      message.data = "set pos 0 0";
      vel_pub.publish(message);

    //   r.sleep();
    // }
  }
};  // namespace head_recovery
