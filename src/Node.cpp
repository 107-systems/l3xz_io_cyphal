/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/ros2_cyphal_bridge/graphs/contributors.
 */

/**************************************************************************************
 * INCcdLUDES
 **************************************************************************************/

#include <ros2_cyphal_bridge/Node.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Node::Node()
: rclcpp::Node("ros2_cyphal_bridge")
{
  declare_parameter("can_iface", "can0");

  RCLCPP_INFO(get_logger(), "configuring CAN bus:\n\tDevice: %s", get_parameter("can_iface").as_string().c_str());

  _can_mgr = std::unique_ptr<CanManager>
    (new CanManager(get_logger(),
                    get_parameter("can_iface").as_string(),
                    [this](CanardFrame const & frame)
                    {
                      RCLCPP_INFO(get_logger(), "CAN frame received");
                    }));

  _io_loop_timer = create_wall_timer
    (std::chrono::milliseconds(IO_LOOP_RATE.count()),
     [this]()
     {
       this->io_loop();
     });

  RCLCPP_INFO(get_logger(), "Node started successfully.");
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Node::io_loop()
{
  auto const now = std::chrono::steady_clock::now();
  auto const io_loop_rate = (now - _prev_io_loop_timepoint);
  if (io_loop_rate > (IO_LOOP_RATE + std::chrono::milliseconds(1)))
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         1000,
                         "io_loop should be called every %ld ms, but is %ld ms instead",
                         IO_LOOP_RATE.count(),
                         std::chrono::duration_cast<std::chrono::milliseconds>(io_loop_rate).count());
  _prev_io_loop_timepoint = now;

}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
