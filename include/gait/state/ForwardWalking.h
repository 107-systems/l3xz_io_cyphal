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

#include "GaitControllerState.h"

#include <map>
#include <list>

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum class LegState {Ground, Air};
typedef std::map<Leg, LegState> LegStateMap;

typedef std::list<LegStateMap> GaitSequence;

static GaitSequence const RIPPLE_GAIT =
{
  /* 1st gait sequence. */
  {
    {Leg::FrontLeft,   LegState::Ground},
    {Leg::FrontRight,  LegState::Air},
    {Leg::MiddleLeft,  LegState::Air},
    {Leg::MiddleRight, LegState::Ground},
    {Leg::BackLeft,    LegState::Ground},
    {Leg::BackRight,   LegState::Ground},
  },
  /* 2nd gait sequence. */
  {
    {Leg::FrontLeft,   LegState::Ground},
    {Leg::FrontRight,  LegState::Ground},
    {Leg::MiddleLeft,  LegState::Ground},
    {Leg::MiddleRight, LegState::Air},
    {Leg::BackLeft,    LegState::Air},
    {Leg::BackRight,   LegState::Ground},
  },
  /* 3rd gait sequence. */
  {
    {Leg::FrontLeft,   LegState::Air},
    {Leg::FrontRight,  LegState::Ground},
    {Leg::MiddleLeft,  LegState::Ground},
    {Leg::MiddleRight, LegState::Ground},
    {Leg::BackLeft,    LegState::Ground},
    {Leg::BackRight,   LegState::Air},
  },
};

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ForwardWalking : public GaitControllerState
{
public:
           ForwardWalking();
  virtual ~ForwardWalking() { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual GaitControllerState * update(GaitControllerStateInput const & input, GaitControllerStateOutput & output) override;

private:
  GaitSequence::const_iterator _current_leg_state;
  float _gait_cycle;

  static float constexpr GAIT_CYCLE_INCREMENT = 0.05f;
};

#endif /* FORWARD_WALKING_H_ */
