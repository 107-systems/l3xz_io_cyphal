/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <sstream>
#include <iostream>

#include <boost/program_options.hpp>

#include <l3xz/driver/sweep/SweepThread.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace l3xz::driver;
using namespace boost::program_options;

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

static std::atomic<int> scan_cnt = 0;

/**************************************************************************************
 * FUNCTIONS
 **************************************************************************************/

void sweep_scan_complete(SweepThread::ScanDataVect const & scan)
{
  std::cout << "Scan #" << scan_cnt << std::endl
            << "[" << std::endl;
  scan_cnt++;

  for (auto [angle_deg, distance_m] : scan)
    std::cout << "(" << angle_deg << "," << distance_m << "), ";

  std::cout << std::endl << "]" << std::endl;
}

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  try
  {
    std::string param_device_name;
    int param_baudrate;
    int param_rpm;
    int param_sample_rate;
    int param_num_scans;

    options_description desc("Allowed options");
    desc.add_options()
      ("help", "Show this help message.")
      ("device", value<std::string>(&param_device_name)->required(), "Sweep device name, i.e. /dev/ttyUSB0.")
      ("baud", value<int>(&param_baudrate)->default_value(115200), "Sweep baud rate.")
      ("rpm",
       value<int>(&param_rpm)->notifier([](int const val)
                                          {
                                            if(val < 0 || val > 10)
                                              throw validation_error(validation_error::invalid_option_value, "rpm", std::to_string(val));
                                          })->required(),
       "Scanner rotational speed (0 - 10 RPM)")
      ("sample-rate",
       value<int>(&param_sample_rate)->notifier([](int const val)
                                                  {
                                                   if(val != 500 && val != 750 && val != 1000)
                                                       throw validation_error(validation_error::invalid_option_value, "sample-rate", std::to_string(val));
                                                  })->required(),
       "Scanner sample rate (measurements / second) (500,750,1000).")
      ("num-scans",
       value<int>(&param_num_scans)->default_value(10),
       "Number of scans before the driver shuts down.")
      ;

    variables_map vm;
    store(command_line_parser(argc, argv).options(desc).run(), vm);

    /**************************************************************************************
     * --help
     **************************************************************************************/

    if (vm.count("help"))
    {
      std::cout << "Usage: sweepctrl [options]\n";
      std::cout << desc;
      return EXIT_SUCCESS;
    }

    notify(vm);

    /**************************************************************************************
     * ... do the scan again ...
     **************************************************************************************/

    try
    {
      SweepThread sweep_thd(param_device_name, param_rpm, param_sample_rate, sweep_scan_complete);
      l3xz::common::threading::ThreadBase::stats(std::cout);

      while (scan_cnt < param_num_scans)
        std::this_thread::sleep_for (std::chrono::milliseconds(10));
    }
    catch(sweep::device_error const & e)
    {
      std::cerr << e.what() << std::endl;
      return EXIT_FAILURE;
    }
  }
  catch(std::exception const & e)
  {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
