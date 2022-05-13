/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/state/ForwardWalking.h>

#include <ros/console.h>

#include <gait/state/StandingState.h>

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

ForwardWalking::ForwardWalking()
: _current_leg_state{RIPPLE_GAIT.cbegin()}
, _gait_cycle{0.0}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void ForwardWalking::onEnter()
{
  ROS_INFO("ForwardWalking ENTER");
}

void ForwardWalking::onExit()
{
  ROS_INFO("ForwardWalking EXIT");
}

GaitControllerState * ForwardWalking::update(GaitControllerInput const & input, GaitControllerOutput & output)
{
  /* TODO: Walk one gait cycle forward. */

  /* TODO:
   *  - calc trajectory for each leg on entry.
   *  - determine current target position dependent on progress within gait cycle.
   *  - determine target joint angles via IK from target position.
   *  - compare target vs current angle and set angle actuators accordingly.
   */

  for (auto [leg, leg_state] : (*_current_leg_state))
  {
    switch (leg)
    {
      case Leg::FrontLeft:
      {
        /* TODO. */
      }
      break;
      case Leg::FrontRight:  break;
      case Leg::MiddleLeft:  break;
      case Leg::MiddleRight: break;
      case Leg::BackLeft:    break;
      case Leg::BackRight:   break;
    }
  }

  /* The gait cycle variable tells us where we
   * are within a single cycle of the complete
   * gait sequence.
   */
  _gait_cycle += GAIT_CYCLE_INCREMENT;

  if (_gait_cycle >= 1.0f)
  {
    /* Reset the gait cycle. */
    _gait_cycle = 0;
    /* One full gait cycle has been completed, advance to the next one. */
    _current_leg_state = std::next(_current_leg_state);
    if (_current_leg_state == RIPPLE_GAIT.cend()) {
      /* Now the one complete cycle of walking forward has been completed, return to the the default state. */
      return new StandingState;
    }
  }

  return this;
}
