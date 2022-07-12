/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <driver/orel20/Orel20.h>

#include <phy/opencyphal/Types.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace driver
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Orel20::Orel20(phy::opencyphal::Node & node, CanardNodeID const orel_node_id)
: _node{node}
, OREL20_NODE_ID{orel_node_id}
, _rpm_val{0}
{
  reg::udral::service::common::Readiness_0_1<3001> readiness;
  readiness.data.value = reg_udral_service_common_Readiness_0_1_ENGAGED;

  if (!_node.publish(readiness, OREL20_NODE_ID))
    printf("[ERROR] Orel20::Orel20: error, could not configure ESC to be ready");
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Orel20::spinOnce()
{
  reg::udral::service::actuator::common::sp::Scalar_0_1<3002> setpoint;
  setpoint.data.value = _rpm_val;

  if (!_node.publish(setpoint, OREL20_NODE_ID))
    printf("[ERROR] Orel20::spinOnce: error, could not publish esc message");
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* driver */
