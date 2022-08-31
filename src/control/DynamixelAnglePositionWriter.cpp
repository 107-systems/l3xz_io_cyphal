/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_io/control/DynamixelAnglePositionWriter.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace control
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

DynamixelAnglePositionWriter::DynamixelAnglePositionWriter()
{ }

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void DynamixelAnglePositionWriter::update(LegJointKey const joint, float const angle_deg)
{
  static std::map<LegJointKey, DynamixelServoName> const LEG_JOINT_KEY_TO_DYNAMIXEL_SERVO_NAME =
  {
    {make_key(Leg::LeftFront,   Joint::Coxa), DynamixelServoName::LeftFront_Coxa},
    {make_key(Leg::LeftMiddle,  Joint::Coxa), DynamixelServoName::LeftMiddle_Coxa},
    {make_key(Leg::LeftBack,    Joint::Coxa), DynamixelServoName::LeftBack_Coxa},
    {make_key(Leg::RightBack,   Joint::Coxa), DynamixelServoName::RightBack_Coxa},
    {make_key(Leg::RightMiddle, Joint::Coxa), DynamixelServoName::RightMiddle_Coxa},
    {make_key(Leg::RightFront,  Joint::Coxa), DynamixelServoName::RightFront_Coxa},
  };

  update(LEG_JOINT_KEY_TO_DYNAMIXEL_SERVO_NAME.at(joint), angle_deg);
}

void DynamixelAnglePositionWriter::update(HeadJointKey const joint, float const angle_deg)
{
  static std::map<HeadJointKey, DynamixelServoName> const HEAD_JOINT_KEY_TO_DYNAMIXEL_SERVO_NAME =
  {
    {HeadJointKey::Pan,  DynamixelServoName::Head_Pan},
    {HeadJointKey::Tilt, DynamixelServoName::Head_Tilt},
  };

  update(HEAD_JOINT_KEY_TO_DYNAMIXEL_SERVO_NAME.at(joint), angle_deg);
}

bool DynamixelAnglePositionWriter::doBulkWrite(dynamixel::SharedMX28 mx28_ctrl)
{
  dynamixel::MX28::AngleDataSet angle_data_set;

  for (auto [id, angle_deg] : _dynamixel_angle_map)
  {
    float const corrected_angle_deg = (angle_deg + 180.0f);
    angle_data_set[id] = corrected_angle_deg;
  }

  return mx28_ctrl->setAngle(angle_data_set);
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void DynamixelAnglePositionWriter::update(DynamixelServoName const name, float const angle_deg)
{
  _dynamixel_angle_map[toServoId(name)] = angle_deg;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* control */
