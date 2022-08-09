/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <glue/l3xz/ELROB2022/SSC32PWMActuator.h>
#include <glue/l3xz/ELROB2022/SSC32PWMActuatorBulkwriter.h>
#include <glue/l3xz/ELROB2022/SSC32ValveActuator.h>
#include <glue/l3xz/ELROB2022/SSC32AnglePositionActuator.h>

#include <IoNode.h>

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void init_ssc32  (driver::SharedSSC32 & ssc32_ctrl);
void deinit_ssc32(driver::SharedSSC32 & ssc32_ctrl);

/**************************************************************************************
 * CONSTANT
 **************************************************************************************/

static std::string const SSC32_DEVICE_NAME = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH05FOBL-if00-port0";
static size_t      const SSC32_BAUDRATE = 115200;

static uint8_t     const OREL20_NODE_ID = 127;

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv) try
{
  rclcpp::init(argc, argv);

  /**************************************************************************************
   * SSC32
   **************************************************************************************/

  auto ssc32_ctrl = std::make_shared<driver::SSC32>(SSC32_DEVICE_NAME, SSC32_BAUDRATE);

  init_ssc32(ssc32_ctrl);
  printf("[INFO] init_ssc32 successfully completed.");

  glue::l3xz::ELROB2022::SSC32PWMActuatorBulkwriter ssc32_pwm_actuator_bulk_driver(ssc32_ctrl);

  glue::l3xz::ELROB2022::SSC32PWMActuator pwm_actuator_valve_front_left_femur  ("L/F Femur",  0, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  glue::l3xz::ELROB2022::SSC32PWMActuator pwm_actuator_valve_front_left_tibia  ("L/F Tibia",  1, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  glue::l3xz::ELROB2022::SSC32PWMActuator pwm_actuator_valve_middle_left_femur ("L/M Femur",  2, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  glue::l3xz::ELROB2022::SSC32PWMActuator pwm_actuator_valve_middle_left_tibia ("L/M Tibia",  3, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  glue::l3xz::ELROB2022::SSC32PWMActuator pwm_actuator_valve_back_left_femur   ("L/B Femur",  4, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  glue::l3xz::ELROB2022::SSC32PWMActuator pwm_actuator_valve_back_left_tibia   ("L/B Tibia",  5, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });

  glue::l3xz::ELROB2022::SSC32PWMActuator pwm_actuator_valve_front_right_femur ("R/F Femur", 16, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  glue::l3xz::ELROB2022::SSC32PWMActuator pwm_actuator_valve_front_right_tibia ("R/F Tibia", 17, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  glue::l3xz::ELROB2022::SSC32PWMActuator pwm_actuator_valve_middle_right_femur("R/M Femur", 18, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  glue::l3xz::ELROB2022::SSC32PWMActuator pwm_actuator_valve_middle_right_tibia("R/M Tibia", 19, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  glue::l3xz::ELROB2022::SSC32PWMActuator pwm_actuator_valve_back_right_femur  ("R/B Femur", 20, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  glue::l3xz::ELROB2022::SSC32PWMActuator pwm_actuator_valve_back_right_tibia  ("R/B Tibia", 21, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });


  glue::l3xz::ELROB2022::SSC32ValveActuator valve_actuator_front_left_femur  ("L/F Femur", pwm_actuator_valve_front_left_femur);
  glue::l3xz::ELROB2022::SSC32ValveActuator valve_actuator_front_left_tibia  ("L/F Tibia", pwm_actuator_valve_front_left_tibia);
  glue::l3xz::ELROB2022::SSC32ValveActuator valve_actuator_middle_left_femur ("L/M Femur", pwm_actuator_valve_middle_left_femur);
  glue::l3xz::ELROB2022::SSC32ValveActuator valve_actuator_middle_left_tibia ("L/M Tibia", pwm_actuator_valve_middle_left_tibia);
  glue::l3xz::ELROB2022::SSC32ValveActuator valve_actuator_back_left_femur   ("L/B Femur", pwm_actuator_valve_back_left_femur);
  glue::l3xz::ELROB2022::SSC32ValveActuator valve_actuator_back_left_tibia   ("L/B Tibia", pwm_actuator_valve_back_left_tibia);

  glue::l3xz::ELROB2022::SSC32ValveActuator valve_actuator_front_right_femur ("R/F Femur", pwm_actuator_valve_front_right_femur);
  glue::l3xz::ELROB2022::SSC32ValveActuator valve_actuator_front_right_tibia ("R/F Tibia", pwm_actuator_valve_front_right_tibia);
  glue::l3xz::ELROB2022::SSC32ValveActuator valve_actuator_middle_right_femur("R/M Femur", pwm_actuator_valve_middle_right_femur);
  glue::l3xz::ELROB2022::SSC32ValveActuator valve_actuator_middle_right_tibia("R/M Tibia", pwm_actuator_valve_middle_right_tibia);
  glue::l3xz::ELROB2022::SSC32ValveActuator valve_actuator_back_right_femur  ("R/B Femur", pwm_actuator_valve_back_right_femur);
  glue::l3xz::ELROB2022::SSC32ValveActuator valve_actuator_back_right_tibia  ("R/B Tibia", pwm_actuator_valve_back_right_tibia);


  auto angle_actuator_left_front_femur       = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("L/F Femur", valve_actuator_front_left_femur,   nullptr);
  auto angle_actuator_left_front_tibia       = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("L/F Tibia", valve_actuator_front_left_tibia,   nullptr);
  auto angle_actuator_left_middle_femur      = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("L/M Femur", valve_actuator_middle_left_femur,  nullptr);
  auto angle_actuator_left_middle_tibia      = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("L/M Tibia", valve_actuator_middle_left_tibia,  nullptr);
  auto angle_actuator_left_back_femur        = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("L/B Femur", valve_actuator_back_left_femur,    nullptr);
  auto angle_actuator_left_back_tibia        = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("L/B Tibia", valve_actuator_back_left_tibia,    nullptr);

  auto angle_actuator_right_front_femur      = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("R/F Femur", valve_actuator_front_right_femur,  nullptr);
  auto angle_actuator_right_front_tibia      = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("R/F Tibia", valve_actuator_front_right_tibia,  nullptr);
  auto angle_actuator_right_middle_femur     = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("R/M Femur", valve_actuator_middle_right_femur, nullptr);
  auto angle_actuator_right_middle_tibia     = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("R/M Tibia", valve_actuator_middle_right_tibia, nullptr);
  auto angle_actuator_right_back_femur       = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("R/B Femur", valve_actuator_back_right_femur,   nullptr);
  auto angle_actuator_right_back_tibia       = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("R/B Tibia", valve_actuator_back_right_tibia,   nullptr);

  /**************************************************************************************
   * STATE
   **************************************************************************************/

  auto io_node = std::make_shared<l3xz::IoNode>
  (
    ssc32_ctrl,
    ssc32_pwm_actuator_bulk_driver
  );

  /**************************************************************************************
   * MAIN LOOP
   **************************************************************************************/

  rclcpp::spin(io_node);
  rclcpp::shutdown();

  printf("[WARNING] STOPPING");

  deinit_ssc32(ssc32_ctrl);

  return EXIT_SUCCESS;
}
catch (std::runtime_error const & err)
{
  printf("[ERROR] Exception (std::runtime_error) caught: %s\nTerminating ...", err.what());
  return EXIT_FAILURE;
}
catch (...)
{
  printf("[ERROR] Unhandled exception caught.\nTerminating ...");
  return EXIT_FAILURE;
}

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void init_ssc32(driver::SharedSSC32 & ssc32_ctrl)
{
  /* Set all servos to neutral position, this
   * means that all valves are turned off.
   */
  for (auto ch = driver::SSC32::MIN_CHANNEL; ch <= driver::SSC32::MAX_CHANNEL; ch++)
    ssc32_ctrl->setPulseWidth(ch, 1500, 50);
}

void deinit_ssc32(driver::SharedSSC32 & ssc32_ctrl)
{
  init_ssc32(ssc32_ctrl);
}
