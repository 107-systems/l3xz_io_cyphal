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

#include <l3xz_io/types/Leg.h>

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


  static CanardNodeID toNodeId(Leg const leg);


  bool  isBumperPressed();
  float femurAngle_deg();
  float tibiaAngle_deg();


private:
  std::atomic<bool> _is_bumper_pressed;
  std::atomic<float> _femur_angle_deg, _tibia_angle_deg;

  bool subscribeTibiaTipBumberMessage(phy::opencyphal::Node & node);
  bool subscribeFemurAngleMessage    (phy::opencyphal::Node & node);
  bool subscribeTibiaAngleMessage    (phy::opencyphal::Node & node);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */

#endif /* GLUE_LEG_CONTROLLER_H_ */
