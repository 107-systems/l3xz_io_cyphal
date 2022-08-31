/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_io/control/dynamixel/DynamixelAnglePositionReader.h>

#include <l3xz_io/control/dynamixel/DynamixelIdList.h>
#include <l3xz_io/control/dynamixel/DynamixelServoName.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace control
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

std::tuple<std::map<LegJointKey, float>, std::map<HeadJointKey, float>> DynamixelAnglePositionReader::doBulkRead(SharedMX28 mx28_ctrl, rclcpp::Logger const logger)
{
  DynamixelMX28::AngleDataSet const angle_data_set = mx28_ctrl->getAngle(control::DYNAMIXEL_ID_LIST);

  std::map<DynamixelServoName, float> dynamixel_angle_position_map;

  for (auto [id, angle_deg] : angle_data_set)
  {
    RCLCPP_DEBUG(logger, "MX28AR ID #%d angle = %.2f", id, angle_deg);
    float const corrected_angle_deg = (angle_deg - 180.0f);

    DynamixelServoName const key = toServoName(id);
    dynamixel_angle_position_map[key] = corrected_angle_deg;
  }

  std::map<LegJointKey, float> const dynamixel_leg_joint_angle_position =
  {
    {make_key(Leg::LeftFront,   Joint::Coxa), dynamixel_angle_position_map.at(DynamixelServoName::LeftFront_Coxa)},
    {make_key(Leg::LeftMiddle,  Joint::Coxa), dynamixel_angle_position_map.at(DynamixelServoName::LeftMiddle_Coxa)},
    {make_key(Leg::LeftBack,    Joint::Coxa), dynamixel_angle_position_map.at(DynamixelServoName::LeftBack_Coxa)},
    {make_key(Leg::RightBack,   Joint::Coxa), dynamixel_angle_position_map.at(DynamixelServoName::RightBack_Coxa)},
    {make_key(Leg::RightMiddle, Joint::Coxa), dynamixel_angle_position_map.at(DynamixelServoName::RightMiddle_Coxa)},
    {make_key(Leg::RightFront,  Joint::Coxa), dynamixel_angle_position_map.at(DynamixelServoName::RightFront_Coxa)},
  };

  std::map<HeadJointKey, float> const dynamixel_head_joint_angle_position =
  {
    {HeadJointKey::Pan,  dynamixel_angle_position_map.at(DynamixelServoName::Head_Pan)},
    {HeadJointKey::Tilt, dynamixel_angle_position_map.at(DynamixelServoName::Head_Tilt)},
  };

  return std::make_tuple(dynamixel_leg_joint_angle_position,
                         dynamixel_head_joint_angle_position);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* control */
