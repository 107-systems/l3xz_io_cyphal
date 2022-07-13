/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_LED_ACTUATOR_H_
#define GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_LED_ACTUATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <phy/opencyphal/Node.hpp>
#include <phy/opencyphal/Types.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue::l3xz::ELROB2022
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class OpenCyphalLEDActuator
{
public:
  OpenCyphalLEDActuator(phy::opencyphal::Node & node)
  : _node{node}
  ,_mode(BlinkMode::Red)
  { }

  enum class BlinkMode : int8_t
  {
    Red = 1,
    Green = 2,
    Amber = 3,
  };

  void setBlinkMode(BlinkMode const mode) {
    _mode = mode;
  }

  void doBulkWrite()
  {
    uavcan::primitive::scalar::Integer8_1_0<ID_LIGHT_MODE> aux_ctrl_light_mode_msg;
    aux_ctrl_light_mode_msg.data.value = static_cast<int8_t>(_mode);
    _node.publish(aux_ctrl_light_mode_msg);
  }

private:
  static CanardPortID const ID_LIGHT_MODE = 2002U;
  phy::opencyphal::Node & _node;
  BlinkMode _mode;
};

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::shared_ptr<OpenCyphalLEDActuator> SharedOpenCyphalLEDActuator;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_OPEN_CYPHAL_LED_ACTUATOR_H_ */
