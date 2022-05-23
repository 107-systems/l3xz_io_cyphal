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

HeadController::HeadController()
: _head_state{new state::Init()}
{
  _head_state->onEnter();
}

HeadController::~HeadController()
{
  delete _head_state;
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

HeadControllerOutput HeadController::update(HeadControllerInput const & input)
{
  HeadControllerOutput output;

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