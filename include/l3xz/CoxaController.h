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

#include "dynamixel/DynamixelController.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class CoxaController
{

public:

  CoxaController(std::unique_ptr<dynamixel::DynamixelController> mx28_ctrl);

  void turnLedOn();
  void turnLedOff();

private:

  std::unique_ptr<dynamixel::DynamixelController> _mx28_ctrl;
  std::vector<uint8_t> _mx28_id_vect;

};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* DYNAMIXEL_COXA_CONTROLLER_H_ */
