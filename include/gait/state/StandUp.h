/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef STANDUP_STATE_H_
#define STANDUP_STATE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "StateBase.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait::state
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class StandUp : public StateBase
{
public:
  virtual ~StandUp() { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual StateBase * update(GaitControllerInput & input, GaitControllerOutput & output) override;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */

#endif /* STANDUP_STATE_H_ */
