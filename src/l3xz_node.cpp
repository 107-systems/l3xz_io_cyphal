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

#include <l3xz/driver/dynamixel/MX28Controller.h>
#include <l3xz/driver/dynamixel/DynamixelController.h>

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
  std::unique_ptr<dynamixel::MX28Controller> mx28_ctrl(new dynamixel::MX28Controller(std::move(dynamixel_ctrl)));

  std::optional<dynamixel::IdVect> opt_id_vect =  mx28_ctrl->discover();
  if (!opt_id_vect)
    ROS_ERROR("Zero MX-28 servos detected.");
  else
  {
    ROS_INFO("Detected Dynamixel MX-28:");
    for (uint8_t id : opt_id_vect.value())
      ROS_INFO("[ID:%d]", id);
  }

  float angle = 0.0f;

  mx28_ctrl->torqueOn(opt_id_vect.value());

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ros::spinOnce();

    mx28_ctrl->turnLedOn(opt_id_vect.value());
    loop_rate.sleep();
    mx28_ctrl->turnLedOff(opt_id_vect.value());
    loop_rate.sleep();

    dynamixel::MX28Controller::AngleDataVect angle_vect_act = mx28_ctrl->getAngle(opt_id_vect.value());
    for (auto [id, angle_deg] : angle_vect_act)
      ROS_INFO("[ID:%03d] Angle Act = %0.02f deg", id, angle_deg);

    dynamixel::MX28Controller::AngleDataVect angle_vect_set = angle_vect_act;
    std::transform(angle_vect_act.begin(),
                   angle_vect_act.end(),
                   angle_vect_set.begin(),
                   [](std::tuple<uint8_t, float> const & in) -> std::tuple<uint8_t, float>
                   {
                     auto [id, angle_set] = in;
                     angle_set += 10.0f;
                     while (angle_set > 360.0f)
                      angle_set -= 360.0f;
                     return std::make_tuple(id, angle_set);
                   });

    for (auto [id, angle_deg] : angle_vect_set)
      ROS_INFO("[ID:%03d] Angle Set = %0.02f deg", id, angle_deg);


    if (!mx28_ctrl->setAngle(angle_vect_set))
      ROS_ERROR("setAngle() failed");
  }

  return EXIT_SUCCESS;
}
