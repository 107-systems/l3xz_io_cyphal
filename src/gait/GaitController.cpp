/**
 * Copyright (c) 2022 LXGaitControllerics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/GaitController.h>

#include <gait/state/InitState.h>

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

GaitController::GaitController()
: _robot_state{new InitState()}
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

void GaitController::update(GaitControllerStateInput const & input, GaitControllerStateOutput & output)
{
  GaitControllerState * next_robot_state = _robot_state->update(input, output);
    
  if (next_robot_state != _robot_state)
  {
    _robot_state->onExit();

    delete _robot_state;
    _robot_state = next_robot_state;
    
    _robot_state->onEnter();
  }
}
