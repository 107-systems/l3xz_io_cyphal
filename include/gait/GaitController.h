/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef GAIT_CONTROLLER_H_
#define GAIT_CONTROLLER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "state/StateBase.h"
#include "GaitControllerInput.h"
#include "GaitControllerOutput.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class GaitController
{
public:
   GaitController();
  ~GaitController();

  void update(GaitControllerInput const & input, GaitControllerOutput & output);

private:
  state::StateBase * _robot_state;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait */

#endif /* GAIT_CONTROLLER_H_ */
