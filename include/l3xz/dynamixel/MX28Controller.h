/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef DYNAMIXEL_COXA_CONTROLLER_H_
#define DYNAMIXEL_COXA_CONTROLLER_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <memory>

#include "DynamixelController.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace dynamixel
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class MX28Controller
{

public:

  MX28Controller(std::unique_ptr<DynamixelController> dyn_ctrl);


  std::optional<IdVect> discover();


  void turnLedOn();
  void turnLedOff();

  typedef std::vector<std::tuple<uint8_t, float>> AngleDataVect;
  AngleDataVect getCurrentPosition();

private:

  std::unique_ptr<DynamixelController> _dyn_ctrl;
  std::vector<uint8_t> _mx28_id_vect;

};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* dynamixel */

#endif /* DYNAMIXEL_COXA_CONTROLLER_H_ */
