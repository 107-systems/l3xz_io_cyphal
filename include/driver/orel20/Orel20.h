/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef DRIVER_OREL20_OREL20_H_
#define DRIVER_OREL20_OREL20_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/esc/RPMCommand.hpp>

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
  Orel20(uint8_t const dronecan_node_id);

  inline void setRPM(uint16_t const rpm_val) { _rpm_val = rpm_val; }

  void spinOnce();

private:
  static unsigned constexpr NODE_MEMORY_POOL_SIZE = 16384;
  uavcan::Node<NODE_MEMORY_POOL_SIZE> _node;
  uavcan::Publisher<uavcan::equipment::esc::RPMCommand> _esc_pub;
  uint16_t _rpm_val;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* driver */

#endif /* DRIVER_OREL20_OREL20_H_ */
