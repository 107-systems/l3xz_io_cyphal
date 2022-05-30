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

#include "StateBase.h"

#include <map>
#include <list>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait::state
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ForwardWalking : public StateBase
{
public:
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual std::tuple<StateBase *, ControllerOutput> update(common::kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output) override;

private:
  float _phase = 0;   // [0, 1]

  static float wrapPhase(const float p) { return (p < 1.0F) ? p : (p - 1.0F); }

  static constexpr float PHASE_INCREMENT = 0.005;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */

#endif /* FORWARD_WALKING_H_ */
