/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef CONST_H_
#define CONST_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <list>

#include "Types.h"

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

static std::list<Leg> const LEG_LIST =
{
  Leg::LeftFront, Leg::LeftMiddle, Leg::LeftBack, Leg::RightFront, Leg::RightMiddle, Leg::RightBack
};

#endif /* CONST_H_ */
