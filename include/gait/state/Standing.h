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

#include "StateBase.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait::state
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Standing : public StateBase
{
public:
  virtual ~Standing() { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual std::tuple<StateBase *, GaitControllerOutput> update(common::kinematic::Engine const & engine, GaitControllerInput const & input, GaitControllerOutput const & prev_output) override;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */

#endif /* STANDING_STATE_H_ */
