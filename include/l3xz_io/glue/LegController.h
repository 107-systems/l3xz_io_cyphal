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

#include <l3xz_io/phy/opencyphal/Node.hpp>

#include <l3xz_io/types/Leg.h>

#include <l3xz_io/phy/opencyphal/Types.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class LegController
{
public:
  LegController(phy::opencyphal::Node & node, rclcpp::Logger const logger);


  bool isHeartbeatTimeout(Leg const leg, std::chrono::seconds const timeout);
  bool isModeOperational (Leg const leg);
  bool isHealthNominal   (Leg const leg);


  bool  isBumperPressed(Leg const leg);
  float femurAngle_deg (Leg const leg);
  float tibiaAngle_deg (Leg const leg);


  CanardNodeID toNodeId(Leg const leg) const;


private:
  std::map<CanardNodeID, Leg> const NODE_ID_2_LEG_MAP;
  std::map<Leg, std::atomic<bool>> _is_bumper_pressed;
  std::map<Leg, std::atomic<float>> _femur_angle_deg, _tibia_angle_deg;

  typedef struct
  {
    uavcan_node_Health_1_0 health;
    uavcan_node_Mode_1_0 mode;
    std::chrono::system_clock::time_point timestamp;
  } THeartbeatData;
  std::mutex _heartbeat_mtx;
  std::map<Leg, THeartbeatData> _heartbeat;


  bool subscribeHeartbeat            (phy::opencyphal::Node & node);
  bool subscribeTibiaTipBumberMessage(phy::opencyphal::Node & node);
  bool subscribeFemurAngleMessage    (phy::opencyphal::Node & node);
  bool subscribeTibiaAngleMessage    (phy::opencyphal::Node & node);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */

#endif /* GLUE_LEG_CONTROLLER_H_ */
