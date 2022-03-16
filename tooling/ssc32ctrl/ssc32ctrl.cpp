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

#include <l3xz/phy/serial/AsyncSerial.h>

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
    int param_move_time_us;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
      ("help", "Show this help message.")
      ("device-name", boost::program_options::value<std::string>(&param_device_name)->required(), "SSC32U servo controller device name.")
      ("baud-rate", boost::program_options::value<int>(&param_baudrate)->default_value(9600), "SSC32U servo controller baud rate.")
      ("move", "Move a single servo.")
      ("channel",
       boost::program_options::value<int>(&param_channel)->notifier([](int const val)
                                                                    {
                                                                      if(val < 0 || val > 31)
                                                                        throw boost::program_options::validation_error(boost::program_options::validation_error::invalid_option_value, "channel", std::to_string(val));
                                                                    }),
       "Lynxmotion SSC32U servo channel (0 - 31).")
      ("pulse-width",
       boost::program_options::value<int>(&param_pulse_width_us)->notifier([](int const val)
                                                                    {
                                                                      if(val < 500 || val > 2500)
                                                                        throw boost::program_options::validation_error(boost::program_options::validation_error::invalid_option_value, "pulse-width", std::to_string(val));
                                                                    }),
       "Servo pulse width / us (500 - 2500).")
      ("move-time",
       boost::program_options::value<int>(&param_move_time_us)->notifier([](int const val)
                                                                    {
                                                                      if(val < 0 || val > 65535)
                                                                        throw boost::program_options::validation_error(boost::program_options::validation_error::invalid_option_value, "move-time", std::to_string(val));
                                                                    }),
       "Servo move time / us (0 - 65535).")
      ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).run(), vm);

    /**************************************************************************************
     * --help
     **************************************************************************************/

    if (vm.count("help"))
    {
      std::cout << "Usage: ssc32ctrl [options]\n";
      std::cout << desc;
      return EXIT_SUCCESS;
    }

    boost::program_options::notify(vm);

    phy::serial::AsyncSerial serial;
    serial.open(param_device_name, param_baudrate);
  
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

      std::stringstream msg;
      msg << "#"
          << param_channel
          << "P"
          << param_pulse_width_us
          << "T"
          << param_move_time_us
          << '\r';
      
      std::string const msg_str(msg.str());
      std::cout << msg_str << std::endl;

      std::vector<uint8_t> const msg_vect(msg_str.begin(), msg_str.end());
      serial.transmit(msg_vect);
    }

    serial.close();
  }
  catch(std::exception const & e)
  {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
