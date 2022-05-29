/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef GAIT_CONTROLLER_H_
#define GAIT_CONTROLLER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "state/StateBase.h"
#include "GaitControllerInput.h"
#include "GaitControllerOutput.h"

#include <common/kinematic/Engine.h>

#include <driver/ssc32/SSC32.h>
#include <driver/orel20/Orel20.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Controller
{
public:
   Controller(driver::SharedSSC32 ssc32_ctrl,
              driver::SharedOrel20 orel20_ctrl,
              std::map<LegJointKey, float> & angle_position_sensor_offset_map,
              bool & is_calibrate_complete);
  ~Controller();

  ControllerOutput update(ControllerInput const & input, ControllerOutput const & prev_output);

private:
  state::StateBase * _robot_state;
  common::kinematic::Engine _kinematic_engine;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait */

#endif /* GAIT_CONTROLLER_H_ */
