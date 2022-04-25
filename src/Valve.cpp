/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <Valve.h>

#include <tuple>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::tuple<Leg, Joint> ValveMapKey;
typedef uint8_t ValveMapValue;
typedef std::map<ValveMapKey,ValveMapValue> ValveMap;

struct valve_map_key_equal : public std::binary_function<ValveMapKey, ValveMapKey, bool>
{
  bool operator()(const ValveMapKey & v0, const ValveMapKey & v1) const
  {
    return (
            std::get<0>(v0) == std::get<0>(v1) &&
            std::get<1>(v0) == std::get<1>(v1)
           );
  }
};

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

static ValveMapKey make_key(Leg const leg, Joint const joint)
{
  return std::tuple(leg, joint);
}

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

static ValveMap const VALVE_MAP =
{
  /* LEFT */
  {make_key(Leg::FrontLeft,   Joint::Femur),  0},
  {make_key(Leg::FrontLeft,   Joint::Tibia),  1},
  {make_key(Leg::MiddleLeft,  Joint::Femur),  2},
  {make_key(Leg::MiddleLeft,  Joint::Tibia),  3},
  {make_key(Leg::BackLeft,    Joint::Femur),  4},
  {make_key(Leg::BackLeft,    Joint::Tibia),  5},
  /* RIGHT */
  {make_key(Leg::FrontRight,  Joint::Femur), 16},
  {make_key(Leg::FrontRight,  Joint::Tibia), 17},
  {make_key(Leg::MiddleRight, Joint::Femur), 18},
  {make_key(Leg::MiddleRight, Joint::Tibia), 19},
  {make_key(Leg::BackRight,   Joint::Femur), 20},
  {make_key(Leg::BackRight,   Joint::Tibia), 21},
};

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Valve::Valve(driver::SharedSSC32 & ssc32)
: _ssc32{ssc32}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Valve::set(Leg const leg, Joint const joint, uint16_t const pulse_width_us)
{
  uint8_t const channel = VALVE_MAP.at(make_key(leg, joint));

  _ssc32->setPulseWidth(channel, pulse_width_us, 0);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
