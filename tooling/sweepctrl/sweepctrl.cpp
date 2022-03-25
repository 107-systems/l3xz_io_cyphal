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
 * FUNCTIONS
 **************************************************************************************/

void sweep_scan_complete(SweepThread::ScanDataVect const & scan)
{
  static int scan_cnt = 0;
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
    SweepThread sweep_thd("/dev/ttyUSB0", 1, 500, sweep_scan_complete);
    l3xz::common::threading::ThreadBase::stats(std::cout);
    /*
    std::cout << "before setup" << std::endl;
    sweep_thd.setup();
    std::cout << "after setup" << std::endl;
    */
    for(;;) std::this_thread::sleep_for (std::chrono::seconds(1));
    /*
    for(;;)
    {
      std::cout << "before loop" << std::endl;
      sweep_thd.loop();
      std::cout << "after loop" << std::endl;
    }
    */
    return EXIT_SUCCESS;
  }
  catch(sweep::device_error const & e)
  {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}

//   try
//   {
//     std::string param_device_name;
//     int param_baudrate;
//     int param_channel;
//     int param_pulse_width_us;
//     int param_move_time_ms;

//     options_description desc("Allowed options");
//     desc.add_options()
//       ("help", "Show this help message.")
//       ("device", value<std::string>(&param_device_name)->required(), "SSC32U servo controller device name, i.e. /dev/ttyUSB0.")
//       ("baud", value<int>(&param_baudrate)->default_value(115200), "SSC32U servo controller baud rate.")
//       ("move", "Move a single servo.")
//       ("channel",
//        value<int>(&param_channel)->notifier([](int const val)
//                                             {
//                                               if(val < 0 || val > 31)
//                                                 throw validation_error(validation_error::invalid_option_value, "channel", std::to_string(val));
//                                             }),
//        "Servo channel (0 - 31).")
//       ("pulse",
//        value<int>(&param_pulse_width_us)->notifier([](int const val)
//                                                    {
//                                                     if(val < 500 || val > 2500)
//                                                         throw validation_error(validation_error::invalid_option_value, "pulse", std::to_string(val));
//                                                    }),
//        "Servo pulse width / us (500 - 2500).")
//       ("time",
//        value<int>(&param_move_time_ms)->notifier([](int const val)
//                                                  {
//                                                   if(val < 0 || val > 65535)
//                                                       throw validation_error(validation_error::invalid_option_value, "time", std::to_string(val));
//                                                  }),
//        "Servo travel time / ms (0 - 65535).")
//       ;

//     variables_map vm;
//     store(command_line_parser(argc, argv).options(desc).run(), vm);

//     /**************************************************************************************
//      * --help
//      **************************************************************************************/

//     if (vm.count("help"))
//     {
//       std::cout << "Usage: ssc32ctrl [options]\n";
//       std::cout << desc;
//       return EXIT_SUCCESS;
//     }

//     notify(vm);

//     SSC32 ssc32_ctrl(param_device_name, param_baudrate);
  
//     /**************************************************************************************
//      * --move
//      **************************************************************************************/

//     if (vm.count("move"))
//     {
//       if (!vm.count("channel")) {
//         std::cerr << "Error, specify servo channel via '--channel'" << std::endl;
//         return EXIT_FAILURE;
//       }
//       if (!vm.count("pulse")) {
//         std::cerr << "Error, specify servo pulse width via '--pulse'" << std::endl;
//         return EXIT_FAILURE;
//       }
//       if (!vm.count("time")) {
//         std::cerr << "Error, specify servo travel time via '--time'" << std::endl;
//         return EXIT_FAILURE;
//       }

//       ssc32_ctrl.setPulseWidth(param_channel, param_pulse_width_us, param_move_time_ms);
//     }
//   }
//   catch(std::exception const & e)
//   {
//     std::cerr << e.what() << std::endl;
//     return EXIT_FAILURE;
//   }
