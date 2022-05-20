/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_SSC32_ANGLE_POSITION_ACTUATOR_H_
#define GLUE_L3XZ_ELROB2022_SSC32_ANGLE_POSITION_ACTUATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <common/actuator/interface/AnglePositionActuator.h>

#include <common/actuator/interface/ValveActuator.h>
#include <common/sensor/interface/AnglePositionSensor.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue::l3xz::ELROB2022
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class SSC32AnglePositionActuator : public common::actuator::interface::AnglePositionActuator
{
public:

  SSC32AnglePositionActuator(std::string const & name,
                             common::actuator::interface::SharedValveActuator valve_ctrl,
                             common::sensor::interface::SharedAnglePositionSensor angle_pos_sensor)
  : AnglePositionActuator(name)
  , _valve_ctrl{valve_ctrl}
  , _angle_pos_sensor{angle_pos_sensor}
  { }

  virtual void set(float const & target_angle_deg) override
  {
    _target_angle_deg = target_angle_deg;

    if (!_angle_pos_sensor->get().has_value())
    {
      _valve_ctrl->set(0.0);
      return;
    }

    float const actual_angle_deg = _angle_pos_sensor->get().value();
    float const angle_diff       = (target_angle_deg - actual_angle_deg);

    static float const ANGLE_DIFF_EPSILON = 2.0f;

    if (angle_diff < ANGLE_DIFF_EPSILON)
    {
      _valve_ctrl->set(0.0f);
      return;
    }

    if (angle_diff < 0.0f)
      _valve_ctrl->set(0.8f);
    else
      _valve_ctrl->set(0.8f);
  }

protected:
  virtual std::optional<float> get() const override
  {
    return _target_angle_deg;
  }

private:
  common::actuator::interface::SharedValveActuator _valve_ctrl;
  common::sensor::interface::SharedAnglePositionSensor _angle_pos_sensor;
  std::optional<float> _target_angle_deg;
};

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::shared_ptr<SSC32AnglePositionActuator> SharedSSC32AnglePositionActuator;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_SSC32_ANGLE_POSITION_ACTUATOR_H_ */
