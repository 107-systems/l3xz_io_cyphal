/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef BACKWARD_WALKING_H_
#define BACKWARD_WALKING_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "RobotState.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class BackwardWalking : public RobotState
{
public:
  virtual ~BackwardWalking() { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual RobotState * update(l3xz::TeleopCommandData const cmd, RobotStateInput & input) override;
};

#endif /* BACKWARD_WALKING_H_ */
