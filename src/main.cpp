/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io_cyphal/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <l3xz_io_cyphal/IoNode.h>

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv) try
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<l3xz::IoNode>());
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
catch (std::runtime_error const & err)
{
  std::cerr << "Exception (std::runtime_error) caught: "
            << err.what() << std::endl
            << "Terminating ..." << std::endl;
  return EXIT_FAILURE;
}
catch (...)
{
  std::cerr << "Unhandled exception caught." << std::endl
            << "Terminating ..." << std::endl;
  return EXIT_FAILURE;
}
