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
                             common::actuator::interface::SharedValveActuator valve_actuator,
                             common::sensor::interface::SharedAnglePositionSensor angle_sensor)
  : AnglePositionActuator(name)
  , _valve_actuator{valve_actuator}
  , _angle_sensor{angle_sensor}
  { }

  virtual void set(float const & target_angle_deg) override
  {
    _target_angle_deg = target_angle_deg;

    if (!_angle_sensor->get().has_value())
    {
      _valve_actuator->set(0.0);
      return;
    }

    float const actual_angle_deg = _angle_sensor->get().value();
    float const angle_diff       = (target_angle_deg - actual_angle_deg);
    float const angle_error      = fabs(angle_diff);

    /* Check if we are within an acceptable error range,
     * if we are then it is perfectly fine to turn off
     * the valve.
     */
    static float const TARGET_ANGLE_EPSILON = 2.0f;
    if (angle_error < TARGET_ANGLE_EPSILON)
    {
      _valve_actuator->set(0.0f);
      return;
    }

    /* If we are outside our acceptable error margins
     * turn the valve in the required position.
     */
    float const kP = 1.0f;
    float const set = kP * angle_diff;
    _valve_actuator->set(set);
  }

protected:
  virtual std::optional<float> get() const override
  {
    return _target_angle_deg;
  }

private:
  common::actuator::interface::SharedValveActuator _valve_actuator;
  common::sensor::interface::SharedAnglePositionSensor _angle_sensor;
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
