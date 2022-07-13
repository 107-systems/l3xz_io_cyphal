/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef CALIBRATE_STATE_H_
#define CALIBRATE_STATE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "StateBase.h"

#include <chrono>

#include <driver/ssc32/SSC32.h>
#include <driver/orel20/Orel20.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait::state
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Calibrate : public StateBase
{
public:
           Calibrate(driver::SharedSSC32 ssc32_ctrl,
                     driver::SharedOrel20 orel20_ctrl,
                     std::map<LegJointKey, float> & angle_position_sensor_offset_map,
                     bool & is_calibration_complete);
  virtual ~Calibrate() { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual std::tuple<StateBase *, ControllerOutput> update(common::kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output) override;

private:
  driver::SharedSSC32 _ssc32_ctrl;
  driver::SharedOrel20 _orel20_ctrl;
  std::map<LegJointKey, float> & _angle_position_sensor_offset_map;
  bool & _is_calibration_complete;
  std::chrono::high_resolution_clock::time_point _start_calibration;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */

#endif /* CALIBRATE_STATE_H_ */
