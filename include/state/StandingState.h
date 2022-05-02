/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef STANDING_STATE_H_
#define STANDING_STATE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "RobotState.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class StandingState : public RobotState
{
public:
  virtual ~StandingState() { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual RobotState * update(TeleopCommandData const cmd, RobotStateInput & input, RobotStateOutput & output) override;
};

#endif /* STANDING_STATE_H_ */
