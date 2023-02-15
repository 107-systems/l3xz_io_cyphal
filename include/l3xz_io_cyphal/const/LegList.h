/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io_cyphal/graphs/contributors.
 */

#ifndef CONST_LEG_LIST_H_
#define CONST_LEG_LIST_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_io_cyphal/types/Leg.h>

/**************************************************************************************
 * CONST
 **************************************************************************************/

static std::list<Leg> const LEG_LIST =
{
  Leg::LeftFront,
  Leg::LeftMiddle,
  Leg::LeftBack,
  Leg::RightBack,
  Leg::RightMiddle,
  Leg::RightFront,
};

#endif /* CONST_LEG_LIST_H_ */
