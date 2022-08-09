/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef DRIVER_OREL20_OREL20_H_
#define DRIVER_OREL20_OREL20_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <phy/opencyphal/Node.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace driver
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Orel20
{
public:

  Orel20(phy::opencyphal::Node & node,
         rclcpp::Logger const logger);

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

} /* driver */

#endif /* DRIVER_OREL20_OREL20_H_ */
