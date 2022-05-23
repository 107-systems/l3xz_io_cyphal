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

#include <gait/state/Standing.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait::state
{

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

StateBase * ForwardWalking::update(GaitControllerInput & input, GaitControllerOutput & output)
{
  /* TODO: Walk one gait::state cycle forward. */

  /* 1st: Move to start position: by placing F/R, M/L a half step forward. */

  /* TODO:
   *  - calc trajectory for each leg on entry.
   *  - determine current target position dependent on progress within gait::state cycle.
   *  - determine target joint angles via IK from target position.
   *  - compare target vs current angle and set angle actuators accordingly.
   */

  for (auto [leg, leg_state] : (*_current_leg_state))
  {
    switch (leg)
    {
      case Leg::LeftFront:
      {
        /* TODO. */
      }
      break;
      case Leg::RightFront:  break;
      case Leg::LeftMiddle:  break;
      case Leg::RightMiddle: break;
      case Leg::LeftBack:    break;
      case Leg::RightBack:   break;
      default: break;
    }
  }

  /* The gait::state cycle variable tells us where we
   * are within a single cycle of the complete
   * gait::state sequence.
   */
  _gait_cycle += GAIT_CYCLE_INCREMENT;

  if (_gait_cycle >= 1.0f)
  {
    /* Reset the gait::state cycle. */
    _gait_cycle = 0;
    /* One full gait::state cycle has been completed, advance to the next one. */
    _current_leg_state = std::next(_current_leg_state);
    if (_current_leg_state == RIPPLE_GAIT.cend()) {
      /* Now the one complete cycle of walking forward has been completed, return to the the default state. */
      return new Standing;
    }
  }

  return this;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */
