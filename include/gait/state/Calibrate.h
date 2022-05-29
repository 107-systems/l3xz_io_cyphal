/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef CALIBRATE_STATE_H_
#define CALIBRATE_STATE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "StateBase.h"

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
           Calibrate(driver::SharedSSC32 ssc32_ctrl, driver::SharedOrel20 orel20_ctrl);
  virtual ~Calibrate() { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual std::tuple<StateBase *, ControllerOutput> update(common::kinematic::Engine const & engine, ControllerInput const & input, ControllerOutput const & prev_output) override;

private:
  driver::SharedSSC32 _ssc32_ctrl;
  driver::SharedOrel20 _orel20_ctrl;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* gait::state */

#endif /* CALIBRATE_STATE_H_ */
