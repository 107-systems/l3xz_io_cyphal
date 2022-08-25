/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_io/glue/PumpController.h>

#include <l3xz_io/glue/OpenCyphalMessageTypes.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

PumpController::PumpController(phy::opencyphal::Node & node, rclcpp::Logger const logger)
: _node{node}
, _logger{logger}
, _rpm_val{0}
{
  OpenCyphalOrel20ReadinessMessage readiness;
  readiness.data.value = reg_udral_service_common_Readiness_0_1_ENGAGED;

  if (!_node.publish(readiness))
    RCLCPP_ERROR(_logger, "could not publish PumpController ESC readiness message");
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void PumpController::setRPM(uint16_t const rpm_val)
{
  uint16_t const MAX_RPM = 8192;
  _rpm_val = std::min(rpm_val, MAX_RPM);
}

void PumpController::doWrite()
{
  OpenCyphalOrel20SetpointMessage setpoint;
  setpoint.data.value = _rpm_val;

  if (!_node.publish(setpoint))
    RCLCPP_ERROR(_logger,"error, could not publish PumpController ESC setpoint message");
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */
