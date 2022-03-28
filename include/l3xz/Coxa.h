/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef COXA_H_
#define COXA_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <optional>

#include <l3xz/Const.h>

#include <l3xz/driver/dynamixel/MX28.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Coxa
{
public:
  Coxa(driver::SharedMX28 & mx28);

  void                 set(Leg const leg, float const angle_deg);
  std::optional<float> get(Leg const leg);

private:
  driver::SharedMX28 _mx28;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* COXA_H_ */
