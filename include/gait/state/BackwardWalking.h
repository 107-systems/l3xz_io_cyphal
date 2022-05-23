/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef BACKWARD_WALKING_H_
#define BACKWARD_WALKING_H_

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

class BackwardWalking : public StateBase
{
public:
  virtual ~BackwardWalking() { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual StateBase * update(common::kinematic::Engine const & engine, GaitControllerInput & input, GaitControllerOutput & output) override;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */

#endif /* BACKWARD_WALKING_H_ */
