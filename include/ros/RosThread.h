/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef ROS_ROS_THREAD_H_
#define ROS_ROS_THREAD_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include "RosBrigdeNode.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class RosThread
{
public:
  RosThread(std::shared_ptr<RosBridgeNode> ros_brigde_node)
  : _ros_thread{}
  , _ros_thread_active{}
  {
    _ros_thread = std::thread([this, &ros_brigde_node]() { this->rosThreadFunc(ros_brigde_node); });
  }

  ~RosThread()
  {
    _ros_thread_active = false;
    _ros_thread.join();
  }

private:

  std::thread _ros_thread;
  std::atomic<bool> _ros_thread_active;

  void rosThreadFunc(std::shared_ptr<RosBridgeNode> ros_brigde_node)
  {
    _ros_thread_active = true;

    while(_ros_thread_active && rclcpp::ok()) {
      rclcpp::spin_some(ros_brigde_node);
    }
  }
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* ROS_ROS_THREAD_H_ */
