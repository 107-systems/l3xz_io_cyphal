/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <string>
#include <iostream>

#include <dynamixel_sdk.h>

#include <boost/program_options.hpp>

#include <l3xz/Coxa.h>
#include <l3xz/driver/dynamixel/MX28.h>
#include <l3xz/driver/dynamixel/Dynamixel.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace l3xz::driver;
using namespace boost::program_options;

/**************************************************************************************
 * CONSTANT
 **************************************************************************************/

static float const DYNAMIXEL_PROTOCOL_VERSION = 2.0f;

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
    float param_target_angle;

    options_description desc("Allowed options");
    desc.add_options()
      ("help", "Produce help message.")
      ("device", value<std::string>(&param_device_name)->required(), "Device name of attached U2D2 USB-to-RS458 converter.")
      ("baud", value<int>(&param_baudrate)->default_value(115200), "Baud rate of attached Dynamixel servos.")
      ("leg",
       value<std::string>(&param_leg)->notifier([](std::string const val)
                                                  {
                                                   if(val != "fr" && val != "fl" && val != "ml" && val != "mr" && val != "bl" && val != "br")
                                                    throw validation_error(validation_error::invalid_option_value, "leg", val);
                                                  })->required(),
       "Leg selection (i.e. \"fr\" = front right")
      ("target-angle", value<float>(&param_target_angle), "Desired target angle in degrees.")
      ("get-angle", "Retrieve current angle of a specific servo.")
      ("set-angle", "Set target angle for a specific servo.")
      ;

    variables_map vm;
    store(command_line_parser(argc, argv).options(desc).run(), vm);

    /**************************************************************************************
     * --help
     **************************************************************************************/

    if (vm.count("help"))
    {
      std::cout << "Usage: dynctrl [options]\n";
      std::cout << desc;
      return EXIT_SUCCESS;
    }

    notify(vm);

    std::shared_ptr<Dynamixel> dynamixel_ctrl = std::make_shared<Dynamixel>(param_device_name, DYNAMIXEL_PROTOCOL_VERSION, param_baudrate);
    std::shared_ptr<MX28> mx28_ctrl = std::make_shared<MX28>(dynamixel_ctrl);
    l3xz::Coxa coxa_ctrl(mx28_ctrl);

    std::map<std::string, l3xz::Leg> const LEG_MAP =
    {
      {std::string("fr"), l3xz::Leg::FrontRight},
      {std::string("fl"), l3xz::Leg::FrontLeft},
      {std::string("mr"), l3xz::Leg::MiddleRight},
      {std::string("ml"), l3xz::Leg::MiddleLeft},
      {std::string("br"), l3xz::Leg::BackRight},
      {std::string("bl"), l3xz::Leg::BackLeft},
    };

    /**************************************************************************************
     * --get-angle
     **************************************************************************************/

    if (vm.count("get-angle"))
    {
      std::optional<float> opt_angle_deg = coxa_ctrl.get(LEG_MAP.at(param_leg));

      if (opt_angle_deg)
      {
        std::cout << "Angle = " << opt_angle_deg.value() << "°" << std::endl;
        return EXIT_SUCCESS;
      }

      std::cerr << "Could not obtain angle." << std::endl;
      return EXIT_FAILURE;
    }

    /**************************************************************************************
     * --set-angle
     **************************************************************************************/

    if (vm.count("set-angle"))
    {
      if (!vm.count("target-angle")) {
        std::cerr << "You need to set a target angle --target-angle" << std::endl;
        return EXIT_FAILURE;
      }

      //mx28_ctrl->torqueOn(LEG_MAP.at(param_leg)); TODO!!

      if (!coxa_ctrl.set(LEG_MAP.at(param_leg), param_target_angle)) {
        std::cerr << "Error setting target angle." << std::endl;
        return EXIT_FAILURE;
      }
    }
  }
  catch(std::exception const & e)
  {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
