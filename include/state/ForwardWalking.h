/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef FORWARD_WALKING_H_
#define FORWARD_WALKING_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "RobotState.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ForwardWalking : public RobotState
{
public:
  virtual ~ForwardWalking() { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual RobotState * update(l3xz::TeleopCommandData const cmd) override;
};

#endif /* FORWARD_WALKING_H_ */
