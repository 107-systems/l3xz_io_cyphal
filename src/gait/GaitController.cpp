/**
 * Copyright (c) 2022 LXControllerics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <gait/GaitController.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <gait/state/Calibrate.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace gait
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Controller::Controller(driver::SharedSSC32 ssc32_ctrl,
                       driver::SharedOrel20 orel20_ctrl,
                       std::map<LegJointKey, float> & angle_position_sensor_offset_map,
                       bool & is_calibrate_complete)
: _robot_state{new state::Calibrate(ssc32_ctrl, orel20_ctrl, angle_position_sensor_offset_map, is_calibrate_complete)}
, _kinematic_engine{}
{
  _robot_state->onEnter();
}

Controller::~Controller()
{
  delete _robot_state;
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

ControllerOutput Controller::update(ControllerInput const & input, ControllerOutput const & prev_output)
{
  auto [next_robot_state, next_output] = _robot_state->update(_kinematic_engine, input, prev_output);
    
  if (next_robot_state != _robot_state)
  {
    _robot_state->onExit();

    delete _robot_state;
    _robot_state = next_robot_state;
    
    _robot_state->onEnter();
  }

  return next_output;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait */