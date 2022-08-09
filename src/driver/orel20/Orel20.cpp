/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_io/driver/orel20/Orel20.h>

#include <l3xz_io/glue/OpenCyphalMessageTypes.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace driver
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Orel20::Orel20(phy::opencyphal::Node & node,
               rclcpp::Logger const logger)
: _node{node}
, _logger{logger}
, _rpm_val{0}
{
  glue::OpenCyphalOrel20ReadinessMessage readiness;
  readiness.data.value = reg_udral_service_common_Readiness_0_1_ENGAGED;

  if (!_node.publish(readiness))
    RCLCPP_ERROR(_logger, "could not publish Orel20 ESC readiness message");
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Orel20::setRPM(uint16_t const rpm_val)
{
  uint16_t const MAX_RPM = 8192;
  _rpm_val = std::min(rpm_val, MAX_RPM);
}

void Orel20::doWrite()
{
  glue::OpenCyphalOrel20SetpointMessage setpoint;
  setpoint.data.value = _rpm_val;

  if (!_node.publish(setpoint))
    RCLCPP_ERROR(_logger,"error, could not publish Orel20 ESC setpoint message");
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* driver */
