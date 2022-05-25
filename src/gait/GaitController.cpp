/**
 * Copyright (c) 2022 LXGaitControllerics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/GaitController.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <gait/state/Init.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

GaitController::GaitController()
: _robot_state{new state::Init()}
, _kinematic_engine{}
{
  _robot_state->onEnter();
}

GaitController::~GaitController()
{
  delete _robot_state;
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

GaitControllerOutput GaitController::update(GaitControllerInput & input, GaitControllerOutput const & prev_output)
{
  auto [next_robot_state, next_output] = _robot_state->update(_kinematic_engine, input, prev_output);
    
  if (next_robot_state != _robot_state)
  {
    _robot_state->onExit();

    delete _robot_state;
    _robot_state = next_robot_state;
    
    _robot_state->onEnter();
  }

  return next_output;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait */