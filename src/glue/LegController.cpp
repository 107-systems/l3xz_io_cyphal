/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_io/glue/LegController.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

LegController::LegController(CanardNodeID const node_id,
                             phy::opencyphal::Node & node,
                             rclcpp::Logger const logger)
: OpenCyphalDevice{node_id, node, logger}
{

}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */
