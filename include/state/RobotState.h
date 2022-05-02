/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef ROBOT_STATE_H_
#define ROBOT_STATE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <Const.h>

#include "RobotStateInput.h"
#include "RobotStateOutput.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class RobotState
{
public:
  virtual ~RobotState() { }
  virtual void onEnter() { }
  virtual void onExit() { }
  virtual RobotState * update(TeleopCommandData const cmd, RobotStateInput & input, RobotStateOutput & output) = 0;
};

#endif /* ROBOT_STATE_H_ */
