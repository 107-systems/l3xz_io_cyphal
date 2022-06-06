/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_ACTUATOR_H_
#define GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_ACTUATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <common/actuator/interface/AnglePositionActuator.h>

#include <functional>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue::l3xz::ELROB2022
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class DynamixelAnglePositionActuator : public common::actuator::interface::AnglePositionActuator
{
public:

  typedef std::function<void(driver::Dynamixel::Id const, float const)> OnChangeFunc;

  DynamixelAnglePositionActuator(std::string const & name, driver::Dynamixel::Id const id, OnChangeFunc func)
  : AnglePositionActuator(name)
  , _id{id}
  , _on_change_func{func}
  { }

  virtual void set(float const & angle_deg) override
  {
    _angle_deg = angle_deg;
    _on_change_func(_id, _angle_deg.value());
  }

protected:
  virtual std::optional<float> get() const override
  {
    return _angle_deg;
  }

private:
  driver::Dynamixel::Id _id;
  OnChangeFunc _on_change_func;
  std::optional<float> _angle_deg;
};

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::shared_ptr<DynamixelAnglePositionActuator> SharedDynamixelAnglePositionActuator;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_DYNAMIXEL_ANGLE_POSITION_ACTUATOR_H_ */
