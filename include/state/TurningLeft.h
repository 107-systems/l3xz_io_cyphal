/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef TURNING_LEFT_H_
#define TURNING_LEFT_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "RobotState.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class TurningLeft : public RobotState
{
public:
  virtual ~TurningLeft() { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual RobotState * update(RobotStateInput const & input, RobotStateOutput & output) override;
};

#endif /* TURNING_LEFT_H_ */
