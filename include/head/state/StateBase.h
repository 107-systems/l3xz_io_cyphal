/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef HEAD_CONTROLLER_STATE_H_
#define HEAD_CONTROLLER_STATE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <tuple>

#include <Const.h>
#include "../HeadControllerInput.h"
#include "../HeadControllerOutput.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace head::state
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class StateBase
{
public:
  virtual ~StateBase() { }
  virtual void onEnter() { }
  virtual void onExit() { }
  virtual std::tuple<StateBase *, ControllerOutput> update(ControllerInput const & input, ControllerOutput const & prev_output) = 0;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* head::state */

#endif /* HEAD_CONTROLLER_STATE_H_ */
