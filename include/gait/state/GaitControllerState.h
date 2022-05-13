/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef GAIT_CONTROLLER_STATE_H_
#define GAIT_CONTROLLER_STATE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "GaitControllerStateInput.h"
#include "GaitControllerStateOutput.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class GaitControllerState
{
public:
  virtual ~GaitControllerState() { }
  virtual void onEnter() { }
  virtual void onExit() { }
  virtual GaitControllerState * update(GaitControllerStateInput const & input, GaitControllerStateOutput & output) = 0;
};

#endif /* GAIT_CONTROLLER_STATE_H_ */
