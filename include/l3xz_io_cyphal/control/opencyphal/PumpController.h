/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io_cyphal/graphs/contributors.
 */

#ifndef DRIVER_OREL20_OREL20_H_
#define DRIVER_OREL20_OREL20_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <l3xz_io_cyphal/phy/opencyphal/Node.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace control
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class PumpController
{
public:

  PumpController(phy::opencyphal::Node & node, rclcpp::Logger const logger);

  void setRPM(uint16_t const rpm_val);
  void doWrite();

private:
  phy::opencyphal::Node & _node;
  rclcpp::Logger const _logger;
  uint16_t _rpm_val;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* control */

#endif /* DRIVER_OREL20_OREL20_H_ */
