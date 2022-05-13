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

#include "GaitControllerState.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class InitState : public GaitControllerState
{
public:
  virtual ~InitState() { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual GaitControllerState * update(GaitControllerStateInput const & input, GaitControllerStateOutput & output) override;
};

#endif /* INIT_STATE_H_ */
