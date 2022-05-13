/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef INIT_STATE_H_
#define INIT_STATE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "RobotState.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class InitState : public RobotState
{
public:
  virtual ~InitState() { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual RobotState * update(RobotStateInput const & input, RobotStateOutput & output) override;
};

#endif /* INIT_STATE_H_ */
