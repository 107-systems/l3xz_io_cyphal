/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef HEAD_CONTROLLER_H_
#define HEAD_CONTROLLER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "state/StateBase.h"
#include "HeadControllerInput.h"
#include "HeadControllerOutput.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace head
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class HeadController
{
public:
   HeadController();
  ~HeadController();

  void update(HeadControllerInput const & input, HeadControllerOutput & output);

private:
  state::StateBase * _head_state;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* head */

#endif /* HEAD_CONTROLLER_H_ */
