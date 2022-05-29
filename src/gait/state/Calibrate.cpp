/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/state/Calibrate.h>

#include <sstream>
#include <iomanip>

#include <ros/ros.h>
#include <ros/console.h>

#include <gait/state/Init.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait::state
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Calibrate::Calibrate(driver::SharedSSC32 ssc32_ctrl,
                     driver::SharedOrel20 orel20_ctrl,
                     std::map<LegJointKey, float> & angle_position_sensor_offset_map)
: _ssc32_ctrl{ssc32_ctrl}
, _orel20_ctrl{orel20_ctrl}
, _angle_position_sensor_offset_map{angle_position_sensor_offset_map}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Calibrate::onEnter()
{
  ROS_INFO("Calibrate ENTER");

  /* Open all hydraulic valves. */
  for (auto ch: SERVO_CHANNEL_LIST)
      _ssc32_ctrl->setPulseWidth(ch, 2000, 50);

  /* Start the hydraulic pump. */
  _orel20_ctrl->setRPM(10);

  /* Capture the start time. */
  _start_calibration = std::chrono::high_resolution_clock::now();
}

void Calibrate::onExit()
{
  /* Close all hydraulic valves. */
  for (auto ch: SERVO_CHANNEL_LIST)
    _ssc32_ctrl->setPulseWidth(ch, 1500, 50);

  /* Stop the hydraulic pump. */
  _orel20_ctrl->setRPM(0);

  ROS_INFO("Calibrate EXIT");
}

std::tuple<StateBase *, ControllerOutput> Calibrate::update(common::kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output)
{
  /* Check how much time as progressed, and
   * if enough time has progressed capture
   * the offset angles and proceed to the
   * next step.
   */
  auto const now = std::chrono::high_resolution_clock::now();
  auto const duration = std::chrono::duration_cast<std::chrono::seconds>(now - _start_calibration);

  if (duration.count() < 10)
    return std::tuple(this, prev_output);

  /* Capture the raw angles which are at this point
   * in time the raw values as reported by the sensors.
   */
  for (auto [leg, joint] : HYDRAULIC_LEG_JOINT_LIST)
    _angle_position_sensor_offset_map.at(make_key(leg, joint)) = input.get_angle_deg(leg, joint);

  /* Print the offset values one time for
   * documentation.
   */
  std::stringstream msg;
  msg << "Left" << std::endl
      << "  Front" << std::endl
      << "    Femur" << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::LeftFront, Joint::Femur)) << std::endl
      << "    Tibia" << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::LeftFront, Joint::Tibia)) << std::endl
      << "  Middle" << std::endl
      << "    Femur" << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::LeftMiddle, Joint::Femur)) << std::endl
      << "    Tibia" << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::LeftMiddle, Joint::Tibia)) << std::endl
      << "  Back" << std::endl
      << "    Femur" << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::LeftBack, Joint::Femur)) << std::endl
      << "    Tibia" << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::LeftBack, Joint::Tibia)) << std::endl
      << "Right" << std::endl
      << "  Front" << std::endl
      << "    Femur" << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::RightFront, Joint::Femur)) << std::endl
      << "    Tibia" << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::RightFront, Joint::Tibia)) << std::endl
      << "  Middle" << std::endl
      << "    Femur" << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::RightMiddle, Joint::Femur)) << std::endl
      << "    Tibia" << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::RightMiddle, Joint::Tibia)) << std::endl
      << "  Back" << std::endl
      << "    Femur" << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::RightBack, Joint::Femur)) << std::endl
      << "    Tibia" << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::RightBack, Joint::Tibia)) << std::endl
      ;

  ROS_INFO("Calibrate::update: captured offset angles ...\n%s", msg.str().c_str());

  return std::tuple(new Init(), prev_output);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */
