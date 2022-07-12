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

#include <memory>

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
         CanardNodeID const orel_node_id);

  inline void setRPM(uint16_t const rpm_val) { _rpm_val = rpm_val; }

  void spinOnce();

private:
  phy::opencyphal::Node & _node;
  CanardNodeID const OREL20_NODE_ID;
  uint16_t _rpm_val;
};

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::shared_ptr<Orel20> SharedOrel20;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* driver */

#endif /* DRIVER_OREL20_OREL20_H_ */
