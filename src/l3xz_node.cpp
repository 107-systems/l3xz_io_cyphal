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

  dynamixel::PortHandler * portHandler = dynamixel::PortHandler::getPortHandler(DYNAMIXEL_DEVICE_NAME.c_str());
  dynamixel::PacketHandler * packetHandler = dynamixel::PacketHandler::getPacketHandler(DYNAMIXEL_PROTOCOL_VERSION);

  if (!portHandler->openPort())
    ROS_FATAL("libdynamixel:openPort failed.");

  if (!portHandler->setBaudRate(DYNAMIXEL_BAUD_RATE))
    ROS_FATAL("libdynamixel:setBaudRate failed.");

  std::vector<uint8_t> servo_id_vect;
  if (int const res = packetHandler->broadcastPing(portHandler, servo_id_vect); res != COMM_SUCCESS)
    ROS_ERROR("%s", packetHandler->getTxRxResult(res));

  ROS_INFO("Detected Dynamixel:");
  for (uint8_t id : servo_id_vect)
    ROS_INFO("[ID:%03d]", id);


  dynamixel::DynamixelController dynamixel_ctrl(*portHandler, *packetHandler);

  uint8_t led_off = 0;
  uint8_t led_on = 1;

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ros::spinOnce();

    dynamixel_ctrl.syncWrite(65, sizeof(led_off), std::make_tuple(1, &led_off));
    loop_rate.sleep();
    dynamixel_ctrl.syncWrite(65, sizeof(led_on), std::make_tuple(1, &led_on));
    loop_rate.sleep();
  }

  portHandler->closePort();

  return EXIT_SUCCESS;
}
