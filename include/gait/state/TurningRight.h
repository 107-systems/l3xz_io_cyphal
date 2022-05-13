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

#include "GaitControllerState.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class TurningRight : public GaitControllerState
{
public:
  virtual ~TurningRight() { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual GaitControllerState * update(GaitControllerInput const & input, GaitControllerOutput & output) override;
};

#endif /* TURNING_RIGHT_H_ */
