/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef DRIVER_SWEEP_SWEEP_THREAD_H_
#define DRIVER_SWEEP_SWEEP_THREAD_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <string>
#include <tuple>
#include <functional>

#include <sweep/sweep.hpp>

#include <l3xz/common/threading/ThreadBase.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::driver
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class SweepThread : common::threading::ThreadBase
{
public:

   SweepThread(std::string const device_name, int const baudrate, int const rotation_speed_rpm, int const sample_rate_Hz);
  ~SweepThread();

  typedef std::tuple<double,double> ScanData;
  typedef std::vector<ScanData> ScanDataVect;
  typedef std::function<void(ScanDataVect const &)> OnScanCompleteCallbackFunc;
  void onScanComplete(OnScanCompleteCallbackFunc func);

protected:

  virtual void setup() override;
  virtual void loop() override;


private:

  sweep::sweep _scanner;
  int _rotation_speed_rpm, _sample_rate_Hz;
  OnScanCompleteCallbackFunc _on_scan_complete_callback;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::driver */

#endif /* DRIVER_SWEEP_SWEEP_THREAD_H_ */
