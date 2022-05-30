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

class Walking : public StateBase
{
public:
  explicit Walking(const bool forward) : _phase_increment(forward ? PHASE_INCREMENT_ABS : -PHASE_INCREMENT_ABS) {}

  virtual void onEnter() override;
  virtual void onExit() override;
  virtual std::tuple<StateBase *, ControllerOutput> update(common::kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output) override;

private:
  const float _phase_increment;
  float _phase = 0;   // [0, 1]

  static float wrapPhase(const float p)
  {
    if (p > 1)
    {
      return p - 1;
    }
    if (p < 0)
    {
      return p + 1;
    }
    return p;
  }

  static constexpr float PHASE_INCREMENT_ABS = 0.003;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */

#endif /* FORWARD_WALKING_H_ */
