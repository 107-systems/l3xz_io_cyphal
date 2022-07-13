/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef HEAD_INIT_STATE_H_
#define HEAD_INIT_STATE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "StateBase.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace head::state
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Init : public StateBase
{
public:
  virtual ~Init() { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual std::tuple<StateBase *, ControllerOutput> update(ControllerInput const & input, ControllerOutput const & prev_output) override;

private:
  static float constexpr INITIAL_ANGLE_EPSILON = 2.0f;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* head::state */

#endif /* HEAD_INIT_STATE_H_ */
