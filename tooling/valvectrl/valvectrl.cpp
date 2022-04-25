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

#include <Valve.h>
#include <driver/ssc32/SSC32.h>

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
    std::string param_leg;
    std::string param_joint;
    int param_pulse_width_us;

    options_description desc("Allowed options");
    desc.add_options()
      ("help", "Show this help message.")
      ("device", value<std::string>(&param_device_name)->required(), "SSC32U servo controller device name, i.e. /dev/ttyUSB0.")
      ("baud", value<int>(&param_baudrate)->default_value(115200), "SSC32U servo controller baud rate.")
      ("leg",
       value<std::string>(&param_leg)->notifier([](std::string const val)
                                                  {
                                                   if(val != "fr" && val != "fl" && val != "ml" && val != "mr" && val != "bl" && val != "br")
                                                    throw validation_error(validation_error::invalid_option_value, "leg", val);
                                                  })->required(),
       "Leg selection (i.e. \"fr\" = front right")
      ("joint",
       value<std::string>(&param_joint)->notifier([](std::string const val)
                                                  {
                                                   if(val != "femur" && val != "tibia")
                                                    throw validation_error(validation_error::invalid_option_value, "joint", val);
                                                  })->required(),
       "Joint selection (can be either \"femur\" or \"tibia\")")
      ("pulse",
       value<int>(&param_pulse_width_us)->notifier([](int const val)
                                                   {
                                                    if(val < 500 || val > 2500)
                                                        throw validation_error(validation_error::invalid_option_value, "pulse", std::to_string(val));
                                                   })->required(),
       "Servo pulse width / us (500 - 2500).")
      ;

    variables_map vm;
    store(command_line_parser(argc, argv).options(desc).run(), vm);

    /**************************************************************************************
     * --help
     **************************************************************************************/

    if (vm.count("help"))
    {
      std::cout << "Usage: valvectrl [options]\n";
      std::cout << desc;
      return EXIT_SUCCESS;
    }

    notify(vm);

    SharedSSC32 ssc32_ctrl = std::make_shared<SSC32>(param_device_name, param_baudrate);
    l3xz::Valve valve_ctrl(ssc32_ctrl);
  
    std::map<std::string, l3xz::Leg> const LEG_MAP =
    {
      {std::string("fr"), l3xz::Leg::FrontRight},
      {std::string("fl"), l3xz::Leg::FrontLeft},
      {std::string("mr"), l3xz::Leg::MiddleRight},
      {std::string("ml"), l3xz::Leg::MiddleLeft},
      {std::string("br"), l3xz::Leg::BackRight},
      {std::string("bl"), l3xz::Leg::BackLeft},
    };

    std::map<std::string, l3xz::Joint> const JOINT_MAP =
    {
      {std::string("femur"), l3xz::Joint::Femur},
      {std::string("tibia"), l3xz::Joint::Tibia},
    };

    valve_ctrl.set(LEG_MAP.at(param_leg), JOINT_MAP.at(param_joint), param_pulse_width_us);
  }
  catch(std::exception const & e)
  {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
