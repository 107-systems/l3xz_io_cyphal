/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef GLUE_OPENCYPHAL_NODE_ID_LIST_H_
#define GLUE_OPENCYPHAL_NODE_ID_LIST_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <vector>

#include <canard.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CONSTANT
 **************************************************************************************/

static std::vector<CanardNodeID> const ALLOWED_NODE_ID_LIST{1,2,3,4,5,6};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */

#endif /* GLUE_OPENCYPHAL_NODE_ID_LIST_H_ */
