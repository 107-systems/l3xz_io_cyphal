/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <string>
#include <sstream>
#include <iostream>

#include <boost/program_options.hpp>

#include <l3xz/driver/ssc32/SSC32.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace driver;
using namespace boost::program_options;

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  try
  {
    std::string param_device_name;
    int param_baudrate;
    int param_channel;
    int param_pulse_width_us;
    int param_move_time_ms;

    options_description desc("Allowed options");
    desc.add_options()
      ("help", "Show this help message.")
      ("device", value<std::string>(&param_device_name)->required(), "SSC32U servo controller device name, i.e. /dev/ttyUSB0.")
      ("baud", value<int>(&param_baudrate)->default_value(115200), "SSC32U servo controller baud rate.")
      ("channel",
       value<int>(&param_channel)->notifier([](int const val)
                                            {
                                              if(val < 0 || val > 31)
                                                throw validation_error(validation_error::invalid_option_value, "channel", std::to_string(val));
                                            })->required(),
       "Servo channel (0 - 31).")
      ("pulse",
       value<int>(&param_pulse_width_us)->notifier([](int const val)
                                                   {
                                                    if(val < 500 || val > 2500)
                                                        throw validation_error(validation_error::invalid_option_value, "pulse", std::to_string(val));
                                                   })->required(),
       "Servo pulse width / us (500 - 2500).")
      ("time",
       value<int>(&param_move_time_ms)->notifier([](int const val)
                                                 {
                                                  if(val < 0 || val > 65535)
                                                      throw validation_error(validation_error::invalid_option_value, "time", std::to_string(val));
                                                 })->default_value(0),
       "Servo travel time / ms (0 - 65535).")
      ;

    variables_map vm;
    store(command_line_parser(argc, argv).options(desc).run(), vm);

    /**************************************************************************************
     * --help
     **************************************************************************************/

    if (vm.count("help"))
    {
      std::cout << "Usage: ssc32ctrl [options]\n";
      std::cout << desc;
      return EXIT_SUCCESS;
    }

    notify(vm);

    SSC32 ssc32_ctrl(param_device_name, param_baudrate);
    ssc32_ctrl.setPulseWidth(param_channel, param_pulse_width_us, param_move_time_ms);
  }
  catch(std::exception const & e)
  {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
