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

#include "GaitControllerState.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait::state
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class StandingState : public GaitControllerState
{
public:
  virtual ~StandingState() { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual GaitControllerState * update(GaitControllerInput const & input, GaitControllerOutput & output) override;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */

#endif /* STANDING_STATE_H_ */
