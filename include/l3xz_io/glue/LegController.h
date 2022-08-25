/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef GLUE_LEG_CONTROLLER_H_
#define GLUE_LEG_CONTROLLER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_io/glue/OpenCyphalDevice.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class LegController: public OpenCyphalDevice
{
public:
  LegController(CanardNodeID const node_id,
                phy::opencyphal::Node & node,
                rclcpp::Logger const logger);


  bool isBumperPressed();

private:
  std::atomic<bool> _is_bumper_pressed;

  bool subscribeTibiaTipBumberMessage(phy::opencyphal::Node & node);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */

#endif /* GLUE_LEG_CONTROLLER_H_ */
