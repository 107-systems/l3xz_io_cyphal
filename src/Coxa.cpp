/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz/Coxa.h>

#include <map>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static std::map<Leg, driver::Dynamixel::Id> COXA_MAP =
{
  {Leg::FrontLeft,   1},
  {Leg::FrontRight,  2},
  {Leg::MiddleLeft,  3},
  {Leg::MiddleRight, 4},
  {Leg::BackLeft,    5},
  {Leg::BackRight,   6},
};

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Coxa::Coxa(driver::SharedMX28 & mx28)
: _mx28{mx28}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Coxa::set(Leg const leg, float const angle_deg)
{
  driver::Dynamixel::Id const id = COXA_MAP.at(leg);
  driver::MX28::AngleData const angle_data = std::make_tuple(id, angle_deg);
  _mx28->setAngle(angle_data);
}

std::optional<float> Coxa::get(Leg const leg)
{
  driver::Dynamixel::Id const id = COXA_MAP.at(leg);
  std::optional<driver::MX28::AngleData> const opt_angle_data = _mx28->getAngle(id);
  if (!opt_angle_data)
    return std::nullopt;

  auto [i,angle_deg] =(*opt_angle_data);
  return angle_deg;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
