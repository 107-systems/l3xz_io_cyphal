/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_SSC32_VALVE_ACTUATOR_H_
#define GLUE_L3XZ_ELROB2022_SSC32_VALVE_ACTUATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <map>

#include <l3xz_io/driver/ssc32/SSC32.h>

#include <l3xz_io/types/LegJointKey.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ValveController
{
public:
   ValveController(driver::SharedSSC32 ssc32_ctrl);
  ~ValveController();

  void set(LegJointKey const key, float const val);
  void doBulkWrite();


  void openAllForCalibAndWrite();

private:
  driver::SharedSSC32 _ssc32_ctrl;
  std::map<uint8_t, uint16_t> _channel_pulse_width_map;
  std::map<LegJointKey, uint8_t> LEG_JOINT_KEY_TO_SSC32_SERVO_ID_MAP;

  void closeAllAndWrite();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */

#endif /* GLUE_L3XZ_ELROB2022_SSC32_VALVE_ACTUATOR_H_ */
