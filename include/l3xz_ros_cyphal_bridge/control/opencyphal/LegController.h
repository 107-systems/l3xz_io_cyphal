/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ros_cyphal_bridge/graphs/contributors.
 */

#ifndef GLUE_LEG_CONTROLLER_H_
#define GLUE_LEG_CONTROLLER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_ros_cyphal_bridge/phy/opencyphal/Node.hpp>

#include <l3xz_ros_cyphal_bridge/types/Leg.h>

#include <l3xz_ros_cyphal_bridge/phy/opencyphal/Types.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace control
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class LegController
{
public:
  LegController(phy::opencyphal::Node & node, rclcpp::Logger const logger);


  bool  isBumperPressed(Leg const leg);
  float femurAngle_deg (Leg const leg);
  float tibiaAngle_deg (Leg const leg);


  static CanardNodeID toNodeId(Leg const leg);
  static Leg          toLeg   (CanardNodeID const node_id);


private:
  std::mutex _mtx;
  std::map<Leg, bool> _is_bumper_pressed;
  std::map<Leg, float> _femur_angle_deg, _tibia_angle_deg;


  bool subscribeTibiaTipBumberMessage(phy::opencyphal::Node & node);
  bool subscribeFemurAngleMessage    (phy::opencyphal::Node & node);
  bool subscribeTibiaAngleMessage    (phy::opencyphal::Node & node);

  static bool isLegControllerId(CanardNodeID const node_id);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* control */

#endif /* GLUE_LEG_CONTROLLER_H_ */
