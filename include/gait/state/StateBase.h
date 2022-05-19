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

#include "../GaitControllerInput.h"
#include "../GaitControllerOutput.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait::state
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class StateBase
{
public:
  virtual ~StateBase() { }
  virtual void onEnter() { }
  virtual void onExit() { }
  virtual StateBase * update(GaitControllerInput & input, GaitControllerOutput & output) = 0;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */

#endif /* GAIT_CONTROLLER_STATE_H_ */
