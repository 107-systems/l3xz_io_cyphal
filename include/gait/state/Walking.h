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

struct LegTraits final
{
  std::uint8_t index;
  bool is_left;
  bool is_front;
  bool is_rear;
};

class Walking : public StateBase
{
public:
  explicit Walking(const bool forward) : _phase_increment(forward ? PHASE_INCREMENT_ABS : -PHASE_INCREMENT_ABS) {}

  virtual void onEnter() override;
  virtual void onExit() override;
  virtual std::tuple<StateBase *, ControllerOutput> update(common::kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output) override;

  [[nodiscard]] static KDL::Vector sampleFootTrajectory(const LegTraits lt, const float phase);
  [[nodiscard]] static LegTraits getLegTraits(const Leg leg);

private:
  static const std::vector<KDL::Vector> FOOT_TRAJECTORY;
  static constexpr float PHASE_INCREMENT_ABS = 0.005;

  const float _phase_increment;
  float _phase = 0;   ///< (-1,+1)
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */

#endif /* FORWARD_WALKING_H_ */
