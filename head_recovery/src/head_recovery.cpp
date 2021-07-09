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
    movement_coords_[0][0] = 15;
    movement_coords_[0][1] = 0;

    movement_coords_[1][0] = -15;
    movement_coords_[1][1] = 0;

    movement_coords_[2][0] = 0;
    movement_coords_[2][1] = 15;

    movement_coords_[3][0] = 0;
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
      pub_ = n.advertise<std_msgs::String>("yarp_rpc_wrapper_write", 10);

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

    if (yarp_ports_.size() < 2)
    {
      ROS_ERROR("There need to be at least two ports specified in the config");
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
      if (i != 2){
        message.data = "write " + yarp_ports_[1] + " set pos 1 " + std::to_string(movement_coords_[i][1]);
        pub_.publish(message);
      }
      ros::Duration(3.0).sleep();
    }
  }
}; // namespace head_recovery
