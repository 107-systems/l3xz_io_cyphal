/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_SSC32_VALVE_ACTUATOR_H_
#define GLUE_L3XZ_ELROB2022_SSC32_VALVE_ACTUATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <map>

#include <l3xz_io/driver/ssc32/SSC32.h>

#include <l3xz_io/types/LegJointKey.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class SSC32ValveWriter
{
public:
  SSC32ValveWriter()
  : _channel_pulse_width_map
    {
      { 0, 1500},
      { 1, 1500},
      { 2, 1500},
      { 3, 1500},
      { 4, 1500},
      { 5, 1500},
      {16, 1500},
      {17, 1500},
      {18, 1500},
      {19, 1500},
      {20, 1500},
      {21, 1500},
    }
  , LEG_JOINT_KEY_TO_SSC32_SERVO_ID_MAP
    {
      {make_key(Leg::LeftFront,   Joint::Femur),  0},
      {make_key(Leg::LeftFront,   Joint::Tibia),  1},
      {make_key(Leg::LeftMiddle,  Joint::Femur),  2},
      {make_key(Leg::LeftMiddle,  Joint::Tibia),  3},
      {make_key(Leg::LeftBack,    Joint::Femur),  4},
      {make_key(Leg::LeftBack,    Joint::Tibia),  5},
      {make_key(Leg::RightFront,  Joint::Femur), 16},
      {make_key(Leg::RightFront,  Joint::Tibia), 17},
      {make_key(Leg::RightMiddle, Joint::Femur), 18},
      {make_key(Leg::RightMiddle, Joint::Tibia), 19},
      {make_key(Leg::RightBack,   Joint::Femur), 20},
      {make_key(Leg::RightBack,   Joint::Tibia), 21},
    }
  { }

  void set(LegJointKey const key, float const val)
  {
    /* Limit value to [-1,+1] which resembles
     * a either fully open or a fully closed
     * valve.
     */
    float limited_val = val;
    if (limited_val >  1.0f) limited_val =  1.0f;
    if (limited_val < -1.0f) limited_val = -1.0f;

    /* Calculate the pulse width. */
    uint16_t const pulse_width_us = static_cast<uint16_t>(limited_val * 500.0f) + 1500;

    /* Check if this is indeed a hydraulic leg
     * joint key.
     */
    bool const is_valid_key = (LEG_JOINT_KEY_TO_SSC32_SERVO_ID_MAP.count(key) > 0);
    if (!is_valid_key)
      return;

    /* Update the channel map. */
    _channel_pulse_width_map.at(LEG_JOINT_KEY_TO_SSC32_SERVO_ID_MAP.at(key)) = pulse_width_us;
  }

  void doBulkWrite(driver::SharedSSC32 ssc32_ctrl)
  {
    for (auto [channel, pulse_width_us] : _channel_pulse_width_map)
      ssc32_ctrl->setPulseWidth(channel, pulse_width_us, 50);
  }


private:
  std::map<uint8_t, uint16_t> _channel_pulse_width_map;
  std::map<LegJointKey, uint8_t> LEG_JOINT_KEY_TO_SSC32_SERVO_ID_MAP;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */

#endif /* GLUE_L3XZ_ELROB2022_SSC32_VALVE_ACTUATOR_H_ */
