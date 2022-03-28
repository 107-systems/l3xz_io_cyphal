/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz/driver/sweep/SweepThread.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::driver
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

SweepThread::SweepThread(std::string const device_name, int const rotation_speed_rpm, int const sample_rate_Hz, OnScanCompleteCallbackFunc func)
: common::threading::ThreadBase{std::string(__PRETTY_FUNCTION__)}
, _scanner(device_name.c_str())
, _rotation_speed_rpm{rotation_speed_rpm}
, _sample_rate_Hz{sample_rate_Hz}
, _on_scan_complete_callback{func}
{
  assert(_rotation_speed_rpm >= MIN_ROTATION_SPEED_RPM && _rotation_speed_rpm <= MAX_ROTATION_SPEED_RPM);
  assert(_sample_rate_Hz == 500 || _sample_rate_Hz == 750 || _sample_rate_Hz == 1000);

  startThread();
}

SweepThread::~SweepThread()
{
  _scanner.stop_scanning();
}

/**************************************************************************************
 * PROTECTED MEMBER FUNCTIONS
 **************************************************************************************/

void SweepThread::setup()
{
  _scanner.set_motor_speed(_rotation_speed_rpm);
  while (!_scanner.get_motor_ready()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
  _scanner.set_sample_rate(_sample_rate_Hz);
  _scanner.start_scanning();
}


void SweepThread::loop()
{
  sweep::scan const scan = _scanner.get_scan();

  ScanDataVect scan_data_vect;
  for (auto [angle_milli_deg, distance_mm, signal_strength] : scan.samples)
  {
    double const angle_deg  = static_cast<double>(angle_milli_deg) / 1000.0;
    double const distance_m = static_cast<double>(distance_mm) / 1000.0;
    scan_data_vect.push_back(std::make_tuple(angle_deg, distance_m));
  }

  if (_on_scan_complete_callback)
    _on_scan_complete_callback(scan_data_vect);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::driver */
