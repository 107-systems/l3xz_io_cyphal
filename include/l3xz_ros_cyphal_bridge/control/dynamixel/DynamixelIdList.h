/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ros_cyphal_bridge/graphs/contributors.
 */

#ifndef GLUE_DYNAMIXEL_ID_LIST_H_
#define GLUE_DYNAMIXEL_ID_LIST_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <dynamixel++/Dynamixel++.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace control
{

/**************************************************************************************
 * CONSTANT
 **************************************************************************************/

static dynamixelplusplus::Dynamixel::IdVect const DYNAMIXEL_ID_LIST{1,2,3,4,5,6,7,8};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* control */

#endif /* GLUE_DYNAMIXEL_ID_LIST_H_ */
