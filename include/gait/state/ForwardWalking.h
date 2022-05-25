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

enum class LegState {Ground, Air};
typedef std::map<Leg, LegState> LegStateMap;

typedef std::list<LegStateMap> GaitSequence;

static GaitSequence const RIPPLE_GAIT =
{
  /* 1st gait::state sequence. */
  {
    {Leg::LeftFront,   LegState::Ground},
    {Leg::RightFront,  LegState::Air},
    {Leg::LeftMiddle,  LegState::Air},
    {Leg::RightMiddle, LegState::Ground},
    {Leg::LeftBack,    LegState::Ground},
    {Leg::RightBack,   LegState::Ground},
  },
  /* 2nd gait::state sequence. */
  {
    {Leg::LeftFront,   LegState::Ground},
    {Leg::RightFront,  LegState::Ground},
    {Leg::LeftMiddle,  LegState::Ground},
    {Leg::RightMiddle, LegState::Air},
    {Leg::LeftBack,    LegState::Air},
    {Leg::RightBack,   LegState::Ground},
  },
  /* 3rd gait::state sequence. */
  {
    {Leg::LeftFront,   LegState::Air},
    {Leg::RightFront,  LegState::Ground},
    {Leg::LeftMiddle,  LegState::Ground},
    {Leg::RightMiddle, LegState::Ground},
    {Leg::LeftBack,    LegState::Ground},
    {Leg::RightBack,   LegState::Air},
  },
};

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ForwardWalking : public StateBase
{
public:
           ForwardWalking();
  virtual ~ForwardWalking() { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual std::tuple<StateBase *, GaitControllerOutput> update(common::kinematic::Engine const & engine, GaitControllerInput & input, GaitControllerOutput const & prev_output) override;

private:
  GaitSequence::const_iterator _current_leg_state;
  float _gait_cycle;

  static float constexpr GAIT_CYCLE_INCREMENT = 0.05f;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */

#endif /* FORWARD_WALKING_H_ */
