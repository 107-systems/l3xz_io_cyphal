/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ros_cyphal_bridge/graphs/contributors.
 */

#ifndef CONTROL_OPEN_CYPHAL_ID_LIST_H_
#define CONTROL_OPEN_CYPHAL_ID_LIST_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <list>

#include <canard.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace control
{

/**************************************************************************************
 * CONSTANT
 **************************************************************************************/

static std::list<CanardNodeID> const OPEN_CYPHAL_NODE_ID_LIST = {1, 2, 3, 4, 5, 6, 10};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* control */

#endif /* CONTROL_OPEN_CYPHAL_ID_LIST_H_ */
