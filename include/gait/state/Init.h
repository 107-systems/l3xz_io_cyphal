/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef INIT_STATE_H_
#define INIT_STATE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "StateBase.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait::state
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
  virtual StateBase * update(common::kinematic::Engine const & engine, GaitControllerInput & input, GaitControllerOutput & output) override;

private:
  static float constexpr INITIAL_COXA_ANGLE  = 180.0f;
  static float constexpr INITIAL_FEMUR_ANGLE =   0.0f;
  static float constexpr INITIAL_TIBIA_ANGLE =   0.0f;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */

#endif /* INIT_STATE_H_ */
