/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef TURNING_RIGHT_H_
#define TURNING_RIGHT_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "RobotState.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class TurningRight : public RobotState
{
public:
  virtual ~TurningRight() { }
  virtual Name name() const override { return RobotState::Name::TurningRight; }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual RobotState * update(TeleopCommandData const cmd, RobotStateInput & input, RobotStateOutput & output) override;
};

#endif /* TURNING_RIGHT_H_ */
