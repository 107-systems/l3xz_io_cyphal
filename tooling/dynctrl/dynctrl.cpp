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

#include <l3xz/driver/dynamixel/MX28.h>
#include <l3xz/driver/dynamixel/Dynamixel.h>

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
    int param_id;
    float param_target_angle;

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
      ("help", "Produce help message.")
      ("device-name", boost::program_options::value<std::string>(&param_device_name)->required(), "Device name of attached U2D2 USB-to-RS458 converter.")
      ("baud-rate", boost::program_options::value<int>(&param_baudrate)->default_value(115200), "Baud rate of attached Dynamixel servos.")
      ("id", boost::program_options::value<int>(&param_id), "Dynamixel servo ID.")
      ("target-angle", boost::program_options::value<float>(&param_target_angle), "Desired target angle in degrees.")
      ("discover", "List the ID of all connected servos.")
      ("get-angle", "Retrieve current angle of a specific servo.")
      ("set-angle", "Set target angle for a specific servo.")
      ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).run(), vm);

    /**************************************************************************************
     * --help
     **************************************************************************************/

    if (vm.count("help"))
    {
      std::cout << "Usage: dynctrl [options]\n";
      std::cout << desc;
      return EXIT_SUCCESS;
    }

    boost::program_options::notify(vm);

    std::shared_ptr<l3xz::driver::Dynamixel> dynamixel_ctrl = std::make_shared<l3xz::driver::Dynamixel>(param_device_name, DYNAMIXEL_PROTOCOL_VERSION, param_baudrate);
    std::unique_ptr<l3xz::driver::MX28> mx28_ctrl(new l3xz::driver::MX28(dynamixel_ctrl));

    /**************************************************************************************
     * --discover
     **************************************************************************************/

    if (vm.count("discover"))
    {
      std::optional<l3xz::driver::Dynamixel::IdVect> opt_id_vect =  mx28_ctrl->discover();

      if (!opt_id_vect) {
        std::cout << "Zero node IDs discovered." << std::endl;
        return EXIT_FAILURE;
      }
      
      for (uint8_t id : opt_id_vect.value()) {
        std::cout << "ID: " << static_cast<int>(id) << std::endl;
      }
      return EXIT_SUCCESS;
    }
  
    /**************************************************************************************
     * --get-angle
     **************************************************************************************/

    if (vm.count("get-angle"))
    {
      if (vm.count("id"))
      {
        std::optional<l3xz::driver::MX28::AngleData> opt_angle = mx28_ctrl->getAngle(param_id);

        if (opt_angle)
        {
          auto [id, angle_deg] = opt_angle.value();
          std::cout << "[ID: " << static_cast<int>(id) << "] Angle = " << angle_deg << "°" << std::endl;
          return EXIT_SUCCESS;
        }

        std::cerr << "Could not obtain angle for ID " << param_id << "." << std::endl;
        return EXIT_FAILURE;
      }
      else
      {
        std::optional<l3xz::driver::Dynamixel::IdVect> opt_id_vect =  mx28_ctrl->discover();

        if (opt_id_vect)
        {
          l3xz::driver::MX28::AngleDataVect angle_vect = mx28_ctrl->getAngle(opt_id_vect.value());
          for (auto [id, angle_deg] : angle_vect) {
            std::cout << "[ID: " << static_cast<int>(id) << "] Angle = " << angle_deg << "°" << std::endl;
          }
          return EXIT_SUCCESS;
        }
        else
        {
          std::cerr << "You need to set a Dynamixel servo node ID with --id" << std::endl;
          return EXIT_FAILURE;
        }
      }
    }

    /**************************************************************************************
     * --set-angle
     **************************************************************************************/

    if (vm.count("set-angle"))
    {
      if (!vm.count("id")) {
        std::cerr << "You need to set a Dynamixel servo node ID with --id" << std::endl;
        return EXIT_FAILURE;
      }
      if (!vm.count("target-angle")) {
        std::cerr << "You need to set a target angle --target-angle" << std::endl;
        return EXIT_FAILURE;
      }

      mx28_ctrl->torqueOn(param_id);

      l3xz::driver::MX28::AngleData const angle_data = std::make_tuple(param_id, param_target_angle);

      if (!mx28_ctrl->setAngle(angle_data)) {
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
