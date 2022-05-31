/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef TURNING_LEFT_H_
#define TURNING_LEFT_H_

#include "StateBase.h"

namespace gait::state
{

class Turning : public StateBase
{
public:
  explicit Turning(const bool left) : _left(left) { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual std::tuple<StateBase *, ControllerOutput> update(common::kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output) override;

private:
  const bool _left;
  float _phase = 0;

  static constexpr float PHASE_INCREMENT = 0.005;
};

} /* gait::state */

#endif /* TURNING_LEFT_H_ */
