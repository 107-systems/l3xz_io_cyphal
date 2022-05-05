/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef ROBOT_H_
#define ROBOT_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "state/RobotState.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Robot
{
public:
   Robot();
  ~Robot();

  void update(RobotStateInput & input, RobotStateOutput & output);

private:
  RobotState * _robot_state;
};

#endif /* ROBOT_H_ */
