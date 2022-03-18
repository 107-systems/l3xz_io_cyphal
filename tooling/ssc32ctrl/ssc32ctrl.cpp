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

using namespace l3xz::driver;
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
      ("device-name", value<std::string>(&param_device_name)->required(), "SSC32U servo controller device name.")
      ("baud-rate", value<int>(&param_baudrate)->default_value(115200), "SSC32U servo controller baud rate.")
      ("move", "Move a single servo.")
      ("channel",
       value<int>(&param_channel)->notifier([](int const val)
                                                                    {
                                                                      if(val < 0 || val > 31)
                                                                        throw validation_error(validation_error::invalid_option_value, "channel", std::to_string(val));
                                                                    }),
       "Lynxmotion SSC32U servo channel (0 - 31).")
      ("pulse-width",
       value<int>(&param_pulse_width_us)->notifier([](int const val)
                                                                    {
                                                                      if(val < 500 || val > 2500)
                                                                        throw validation_error(validation_error::invalid_option_value, "pulse-width", std::to_string(val));
                                                                    }),
       "Servo pulse width / us (500 - 2500).")
      ("move-time",
       value<int>(&param_move_time_ms)->notifier([](int const val)
                                                                    {
                                                                      if(val < 0 || val > 65535)
                                                                        throw validation_error(validation_error::invalid_option_value, "move-time", std::to_string(val));
                                                                    }),
       "Servo move time / ms (0 - 65535).")
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
  
    /**************************************************************************************
     * --move
     **************************************************************************************/

    if (vm.count("move"))
    {
      if (!vm.count("channel")) {
        std::cerr << "Error, specify servo channel via '--channel'" << std::endl;
        return EXIT_FAILURE;
      }
      if (!vm.count("pulse-width")) {
        std::cerr << "Error, specify servo pulse width via '--pulse-width'" << std::endl;
        return EXIT_FAILURE;
      }
      if (!vm.count("move-time")) {
        std::cerr << "Error, specify servo move time via '--move-time'" << std::endl;
        return EXIT_FAILURE;
      }

      ssc32_ctrl.setPulseWidth(param_channel, param_pulse_width_us, param_move_time_ms);
    }
  }
  catch(std::exception const & e)
  {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
