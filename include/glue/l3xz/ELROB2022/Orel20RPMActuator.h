/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_OREL20_RPM_ACTUATOR_H_
#define GLUE_L3XZ_ELROB2022_OREL20_RPM_ACTUATOR_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <common/actuator/interface/RPMActuator.h>

#include <driver/orel20/Orel20.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue::l3xz::ELROB2022
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Orel20RPMActuator : public common::actuator::interface::RPMActuator
{
public:
  Orel20RPMActuator(std::string const & name, driver::Orel20 & orel20_ctrl)
  : RPMActuator(name)
  , _orel20_ctrl{orel20_ctrl}
  { }
  virtual ~Orel20RPMActuator() { }

  virtual void set(uint32_t const & val) override
  {
    uint32_t const MAX_RPM = 1000;
    uint16_t const rpm = std::min(val, MAX_RPM);
    _orel20_ctrl.setRPM(rpm);
    _rpm = rpm;
  }

  void doWrite()
  {
    _orel20_ctrl.spinOnce();
  }

protected:
  virtual std::optional<uint32_t> get() const override
  {
    return _rpm;
  }

private:
  driver::Orel20 & _orel20_ctrl;
  std::optional<uint32_t> _rpm;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_OREL20_RPM_ACTUATOR_H_ */
