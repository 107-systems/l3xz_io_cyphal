/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <dynamixel_sdk.h>

#include <l3xz/CoxaController.h>
#include <l3xz/dynamixel/DynamixelController.h>

/**************************************************************************************
 * CONSTANT
 **************************************************************************************/

static std::string const DYNAMIXEL_DEVICE_NAME = "/dev/ttyUSB0";
static float       const DYNAMIXEL_PROTOCOL_VERSION = 2.0f;
static int         const DYNAMIXEL_BAUD_RATE = 115200;

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "l3xz");

  ros::NodeHandle node_handle;

  std::unique_ptr<dynamixel::DynamixelController> dynamixel_ctrl(new dynamixel::DynamixelController(DYNAMIXEL_DEVICE_NAME, DYNAMIXEL_PROTOCOL_VERSION, DYNAMIXEL_BAUD_RATE));
  std::unique_ptr<l3xz::CoxaController> coxa_ctrl(new l3xz::CoxaController(std::move(dynamixel_ctrl)));

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ros::spinOnce();

    coxa_ctrl->turnLedOn();
    loop_rate.sleep();
    coxa_ctrl->turnLedOff();
    loop_rate.sleep();

    /*
    if (auto [err, position_tuple] = dynamixel_ctrl->syncRead(132, 4, 1); err == dynamixel::Error::None)
    {
      auto [id, position] = position_tuple;
      if (position)
        ROS_INFO("[ID:%03d] Present Position (Reg 132): %d", id, position.value());
    }
    */
  }

  return EXIT_SUCCESS;
}
