/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef GAIT_CONTROLLER_H_
#define GAIT_CONTROLLER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "state/GaitControllerState.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class GaitController
{
public:
   GaitController();
  ~GaitController();

  void update(GaitControllerStateInput const & input, GaitControllerStateOutput & output);

private:
  GaitControllerState * _robot_state;
};

#endif /* GAIT_CONTROLLER_H_ */
