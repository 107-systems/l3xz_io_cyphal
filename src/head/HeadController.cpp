/**
 * Copyright (c) 2022 LXHeadControllerics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <head/HeadController.h>

#include <head/state/Init.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace head
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Controller::Controller()
: _head_state{new state::Init()}
{
  _head_state->onEnter();
}

Controller::~Controller()
{
  delete _head_state;
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

ControllerOutput Controller::update(ControllerInput const & input)
{
  ControllerOutput output;

  state::StateBase * next_head_state = _head_state->update(input, output);
    
  if (next_head_state != _head_state)
  {
    _head_state->onExit();

    delete _head_state;
    _head_state = next_head_state;
    
    _head_state->onEnter();
  }

  return output;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* head */