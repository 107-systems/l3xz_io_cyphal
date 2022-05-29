/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <map>
#include <string>
#include <thread>
#include <chrono>
#include <sstream>
#include <functional>

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Twist.h>

#include <dynamixel_sdk.h>

#include <Types.h>
#include <Const.h>

#include <gait/GaitController.h>
#include <head/HeadController.h>

#include <driver/ssc32/SSC32.h>
#include <driver/orel20/Orel20.h>
#include <driver/dynamixel/MX28.h>
#include <driver/dynamixel/Dynamixel.h>

#include <phy/opencyphal/Types.h>
#include <phy/opencyphal/Node.hpp>
#include <phy/opencyphal/SocketCAN.h>

#include <glue/l3xz/ELROB2022/Const.h>
#include <glue/l3xz/ELROB2022/SSC32PWMActuator.h>
#include <glue/l3xz/ELROB2022/SSC32PWMActuatorBulkwriter.h>
#include <glue/l3xz/ELROB2022/SSC32ValveActuator.h>
#include <glue/l3xz/ELROB2022/SSC32AnglePositionActuator.h>
#include <glue/l3xz/ELROB2022/DynamixelAnglePositionSensor.h>
#include <glue/l3xz/ELROB2022/DynamixelAnglePositionSensorBulkReader.h>
#include <glue/l3xz/ELROB2022/DynamixelAnglePositionActuator.h>
#include <glue/l3xz/ELROB2022/DynamixelAnglePositionActuatorBulkWriter.h>
#include <glue/l3xz/ELROB2022/OpenCyphalAnglePositionSensor.h>
#include <glue/l3xz/ELROB2022/OpenCyphalAnglePositionSensorBulkReader.h>
#include <glue/l3xz/ELROB2022/OpenCyphalBumperSensor.h>
#include <glue/l3xz/ELROB2022/OpenCyphalBumperSensorBulkReader.h>
#include <glue/l3xz/ELROB2022/Orel20RPMActuator.h>

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

bool init_dynamixel  (driver::SharedMX28 & mx28_ctrl);
void deinit_dynamixel(driver::SharedMX28 & mx28_ctrl);

bool init_open_cyphal(phy::opencyphal::Node & open_cyphal_node,
                      glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensorBulkReader & open_cyphal_angle_position_sensor_bulk_reader,
                      glue::l3xz::ELROB2022::OpenCyphalBumperSensorBulkReader & open_cyphal_bumper_sensor_bulk_reader);

void deinit_orel20(driver::SharedOrel20 orel20_ctrl);

void init_ssc32  (driver::SharedSSC32 & ssc32_ctrl);
void deinit_ssc32(driver::SharedSSC32 & ssc32_ctrl);

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr & msg, TeleopCommandData & teleop_cmd_data);

/**************************************************************************************
 * CONSTANT
 **************************************************************************************/

static std::string const DYNAMIXEL_DEVICE_NAME = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4NNZ55-if00-port0";
static float       const DYNAMIXEL_PROTOCOL_VERSION = 2.0f;
static int         const DYNAMIXEL_BAUD_RATE = 115200;

static std::string const SSC32_DEVICE_NAME = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH05FOBL-if00-port0";
static size_t      const SSC32_BAUDRATE = 115200;

static uint8_t     const OPEN_CYPHAL_THIS_NODE_ID = 0;
static uint8_t     const DRONECAN_THIS_NODE_ID = 127;

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv) try
{
  ros::init(argc, argv, "l3xz");

  ros::NodeHandle node_hdl;

  /**************************************************************************************
   * DYNAMIXEL
   **************************************************************************************/

  auto dynamixel_ctrl = std::make_shared<driver::Dynamixel>(DYNAMIXEL_DEVICE_NAME, DYNAMIXEL_PROTOCOL_VERSION, DYNAMIXEL_BAUD_RATE);
  auto mx28_ctrl = std::make_shared<driver::MX28>(dynamixel_ctrl);

  if (!init_dynamixel(mx28_ctrl))
    ROS_ERROR("init_dynamixel failed.");
  ROS_INFO("init_dynamixel successfully completed.");

  auto angle_sensor_left_front_coxa   = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionSensor>("L/F Coxa");
  auto angle_sensor_left_middle_coxa  = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionSensor>("L/M Coxa");
  auto angle_sensor_left_back_coxa    = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionSensor>("L/B Coxa");
  auto angle_sensor_right_back_coxa   = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionSensor>("R/B Coxa");
  auto angle_sensor_right_middle_coxa = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionSensor>("R/M Coxa");
  auto angle_sensor_right_front_coxa  = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionSensor>("R/F Coxa");
  auto angle_sensor_sensor_head_pan   = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionSensor>("HEAD Pan");
  auto angle_sensor_sensor_head_tilt  = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionSensor>("HEAD Tilt");

  glue::l3xz::ELROB2022::DynamixelAnglePositionSensorBulkReader dynamixel_angle_position_sensor_bulk_reader
  (
    mx28_ctrl,
    angle_sensor_left_front_coxa,
    angle_sensor_left_middle_coxa,
    angle_sensor_left_back_coxa,
    angle_sensor_right_back_coxa,
    angle_sensor_right_middle_coxa,
    angle_sensor_right_front_coxa,
    angle_sensor_sensor_head_pan,
    angle_sensor_sensor_head_tilt
  );

  glue::l3xz::ELROB2022::DynamixelAnglePositionActuatorBulkWriter dynamixel_angle_position_actuator_bulk_writer(mx28_ctrl);

  auto angle_actuator_left_front_coxa   = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("L/F Coxa",  1, [&dynamixel_angle_position_actuator_bulk_writer](driver::Dynamixel::Id const id, float const angle_deg) { dynamixel_angle_position_actuator_bulk_writer.update(id, angle_deg); });
  auto angle_actuator_left_middle_coxa  = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("L/M Coxa",  2, [&dynamixel_angle_position_actuator_bulk_writer](driver::Dynamixel::Id const id, float const angle_deg) { dynamixel_angle_position_actuator_bulk_writer.update(id, angle_deg); });
  auto angle_actuator_left_back_coxa    = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("L/B Coxa",  3, [&dynamixel_angle_position_actuator_bulk_writer](driver::Dynamixel::Id const id, float const angle_deg) { dynamixel_angle_position_actuator_bulk_writer.update(id, angle_deg); });
  auto angle_actuator_right_back_coxa   = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("R/B Coxa",  4, [&dynamixel_angle_position_actuator_bulk_writer](driver::Dynamixel::Id const id, float const angle_deg) { dynamixel_angle_position_actuator_bulk_writer.update(id, angle_deg); });
  auto angle_actuator_right_middle_coxa = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("R/M Coxa",  5, [&dynamixel_angle_position_actuator_bulk_writer](driver::Dynamixel::Id const id, float const angle_deg) { dynamixel_angle_position_actuator_bulk_writer.update(id, angle_deg); });
  auto angle_actuator_right_front_coxa  = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("R/F Coxa",  6, [&dynamixel_angle_position_actuator_bulk_writer](driver::Dynamixel::Id const id, float const angle_deg) { dynamixel_angle_position_actuator_bulk_writer.update(id, angle_deg); });
  auto angle_actuator_sensor_head_pan   = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("HEAD Pan",  7, [&dynamixel_angle_position_actuator_bulk_writer](driver::Dynamixel::Id const id, float const angle_deg) { dynamixel_angle_position_actuator_bulk_writer.update(id, angle_deg); });
  auto angle_actuator_sensor_head_tilt  = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("HEAD Tilt", 8, [&dynamixel_angle_position_actuator_bulk_writer](driver::Dynamixel::Id const id, float const angle_deg) { dynamixel_angle_position_actuator_bulk_writer.update(id, angle_deg); });

  /**************************************************************************************
   * OPENCYPHAL
   **************************************************************************************/

  phy::opencyphal::SocketCAN open_cyphal_can_if("can0", false);
  phy::opencyphal::Node open_cyphal_node(OPEN_CYPHAL_THIS_NODE_ID, open_cyphal_can_if);

  auto angle_sensor_left_front_femur   = std::make_shared<glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensor>("L/F Femur");
  auto angle_sensor_left_front_tibia   = std::make_shared<glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensor>("L/F Tibia");
  auto angle_sensor_left_middle_femur  = std::make_shared<glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensor>("L/M Femur");
  auto angle_sensor_left_middle_tibia  = std::make_shared<glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensor>("L/M Tibia");
  auto angle_sensor_left_back_femur    = std::make_shared<glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensor>("L/B Femur");
  auto angle_sensor_left_back_tibia    = std::make_shared<glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensor>("L/B Tibia");

  auto angle_sensor_right_back_femur   = std::make_shared<glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensor>("R/B Femur");
  auto angle_sensor_right_back_tibia   = std::make_shared<glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensor>("R/B Tibia");
  auto angle_sensor_right_middle_femur = std::make_shared<glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensor>("R/M Femur");
  auto angle_sensor_right_middle_tibia = std::make_shared<glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensor>("R/M Tibia");
  auto angle_sensor_right_front_femur  = std::make_shared<glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensor>("R/F Femur");
  auto angle_sensor_right_front_tibia  = std::make_shared<glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensor>("R/F Tibia");

  glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensorBulkReader open_cyphal_angle_position_sensor_bulk_reader
  (
    angle_sensor_left_front_femur,
    angle_sensor_left_front_tibia,
    angle_sensor_left_middle_femur,
    angle_sensor_left_middle_tibia,
    angle_sensor_left_back_femur,
    angle_sensor_left_back_tibia,
    angle_sensor_right_back_femur,
    angle_sensor_right_back_tibia,
    angle_sensor_right_middle_femur,
    angle_sensor_right_middle_tibia,
    angle_sensor_right_front_femur,
    angle_sensor_right_front_tibia
  );

  auto tibia_tip_bumper_left_front   = std::make_shared<glue::l3xz::ELROB2022::OpenCyphalBumperSensor>("L/F");
  auto tibia_tip_bumper_left_middle  = std::make_shared<glue::l3xz::ELROB2022::OpenCyphalBumperSensor>("L/M");
  auto tibia_tip_bumper_left_back    = std::make_shared<glue::l3xz::ELROB2022::OpenCyphalBumperSensor>("L/B");
  auto tibia_tip_bumper_right_back   = std::make_shared<glue::l3xz::ELROB2022::OpenCyphalBumperSensor>("R/B");
  auto tibia_tip_bumper_right_middle = std::make_shared<glue::l3xz::ELROB2022::OpenCyphalBumperSensor>("R/M");
  auto tibia_tip_bumper_right_front  = std::make_shared<glue::l3xz::ELROB2022::OpenCyphalBumperSensor>("R/F");

  glue::l3xz::ELROB2022::OpenCyphalBumperSensorBulkReader open_cyphal_bumper_sensor_bulk_reader
  (
    tibia_tip_bumper_left_front,
    tibia_tip_bumper_left_middle,
    tibia_tip_bumper_left_back,
    tibia_tip_bumper_right_back,
    tibia_tip_bumper_right_middle,
    tibia_tip_bumper_right_front
  );

  if (!init_open_cyphal(open_cyphal_node,
                        open_cyphal_angle_position_sensor_bulk_reader,
                        open_cyphal_bumper_sensor_bulk_reader))
  {
    ROS_ERROR("init_open_cyphal failed.");
  }
  ROS_INFO("init_open_cyphal successfully completed.");

  /**************************************************************************************
   * OREL 20 / DRONECAN
   **************************************************************************************/

  auto orel20_ctrl = std::make_shared<driver::Orel20>(DRONECAN_THIS_NODE_ID);

  glue::l3xz::ELROB2022::Orel20RPMActuator orel20_rpm_actuator("Pump", orel20_ctrl);

  /**************************************************************************************
   * SSC32
   **************************************************************************************/

  auto ssc32_ctrl = std::make_shared<driver::SSC32>(SSC32_DEVICE_NAME, SSC32_BAUDRATE);

  init_ssc32(ssc32_ctrl);
  ROS_INFO("init_ssc32 successfully completed.");

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


  auto angle_actuator_left_front_femur       = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("L/F Femur", valve_actuator_front_left_femur,   angle_sensor_left_front_femur);
  auto angle_actuator_left_front_tibia       = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("L/F Tibia", valve_actuator_front_left_tibia,   angle_sensor_left_front_tibia);
  auto angle_actuator_left_middle_femur      = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("L/M Femur", valve_actuator_middle_left_femur,  angle_sensor_left_middle_femur);
  auto angle_actuator_left_middle_tibia      = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("L/M Tibia", valve_actuator_middle_left_tibia,  angle_sensor_left_middle_tibia);
  auto angle_actuator_left_back_femur        = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("L/B Femur", valve_actuator_back_left_femur,    angle_sensor_left_back_femur);
  auto angle_actuator_left_back_tibia        = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("L/B Tibia", valve_actuator_back_left_tibia,    angle_sensor_left_back_tibia);

  auto angle_actuator_right_front_femur      = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("R/F Femur", valve_actuator_front_right_femur,  angle_sensor_right_front_femur);
  auto angle_actuator_right_front_tibia      = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("R/F Tibia", valve_actuator_front_right_tibia,  angle_sensor_right_front_tibia);
  auto angle_actuator_right_middle_femur     = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("R/M Femur", valve_actuator_middle_right_femur, angle_sensor_right_middle_femur);
  auto angle_actuator_right_middle_tibia     = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("R/M Tibia", valve_actuator_middle_right_tibia, angle_sensor_right_middle_tibia);
  auto angle_actuator_right_back_femur       = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("R/B Femur", valve_actuator_back_right_femur,   angle_sensor_right_back_femur);
  auto angle_actuator_right_back_tibia       = std::make_shared<glue::l3xz::ELROB2022::SSC32AnglePositionActuator>("R/B Tibia", valve_actuator_back_right_tibia,   angle_sensor_right_back_tibia);

  /**************************************************************************************
   * ALL ANGLE POSITION ACTUATORS
   **************************************************************************************/

  std::map<LegJointKey, common::actuator::interface::SharedAnglePositionActuator> angle_position_actuator_map =
  {
    {make_key(Leg::LeftFront,   Joint::Coxa),  angle_actuator_left_front_coxa},
    {make_key(Leg::LeftFront,   Joint::Femur), angle_actuator_left_front_femur},
    {make_key(Leg::LeftFront,   Joint::Tibia), angle_actuator_left_front_tibia},
    {make_key(Leg::LeftMiddle,  Joint::Coxa),  angle_actuator_left_middle_coxa},
    {make_key(Leg::LeftMiddle,  Joint::Femur), angle_actuator_left_middle_femur},
    {make_key(Leg::LeftMiddle,  Joint::Tibia), angle_actuator_left_middle_tibia},
    {make_key(Leg::LeftBack,    Joint::Coxa),  angle_actuator_left_back_coxa},
    {make_key(Leg::LeftBack,    Joint::Femur), angle_actuator_left_back_femur},
    {make_key(Leg::LeftBack,    Joint::Tibia), angle_actuator_left_back_tibia},
    {make_key(Leg::RightFront,  Joint::Coxa),  angle_actuator_right_front_coxa},
    {make_key(Leg::RightFront,  Joint::Femur), angle_actuator_right_front_femur},
    {make_key(Leg::RightFront,  Joint::Tibia), angle_actuator_right_front_tibia},
    {make_key(Leg::RightMiddle, Joint::Coxa),  angle_actuator_right_middle_coxa},
    {make_key(Leg::RightMiddle, Joint::Femur), angle_actuator_right_middle_femur},
    {make_key(Leg::RightMiddle, Joint::Tibia), angle_actuator_right_middle_tibia},
    {make_key(Leg::RightBack,   Joint::Coxa),  angle_actuator_right_back_coxa},
    {make_key(Leg::RightBack,   Joint::Femur), angle_actuator_right_back_femur},
    {make_key(Leg::RightBack,   Joint::Tibia), angle_actuator_right_back_tibia}
  };

  std::map<LegJointKey, common::sensor::interface::SharedAnglePositionSensor> angle_position_sensor_map =
  {
    {make_key(Leg::LeftFront,   Joint::Coxa),  angle_sensor_left_front_coxa},
    {make_key(Leg::LeftFront,   Joint::Femur), angle_sensor_left_front_femur},
    {make_key(Leg::LeftFront,   Joint::Tibia), angle_sensor_left_front_tibia},
    {make_key(Leg::LeftMiddle,  Joint::Coxa),  angle_sensor_left_middle_coxa},
    {make_key(Leg::LeftMiddle,  Joint::Femur), angle_sensor_left_middle_femur},
    {make_key(Leg::LeftMiddle,  Joint::Tibia), angle_sensor_left_middle_tibia},
    {make_key(Leg::LeftBack,    Joint::Coxa),  angle_sensor_left_back_coxa},
    {make_key(Leg::LeftBack,    Joint::Femur), angle_sensor_left_back_femur},
    {make_key(Leg::LeftBack,    Joint::Tibia), angle_sensor_left_back_tibia},
    {make_key(Leg::RightFront,  Joint::Coxa),  angle_sensor_right_front_coxa},
    {make_key(Leg::RightFront,  Joint::Femur), angle_sensor_right_front_femur},
    {make_key(Leg::RightFront,  Joint::Tibia), angle_sensor_right_front_tibia},
    {make_key(Leg::RightMiddle, Joint::Coxa),  angle_sensor_right_middle_coxa},
    {make_key(Leg::RightMiddle, Joint::Femur), angle_sensor_right_middle_femur},
    {make_key(Leg::RightMiddle, Joint::Tibia), angle_sensor_right_middle_tibia},
    {make_key(Leg::RightBack,   Joint::Coxa),  angle_sensor_right_back_coxa},
    {make_key(Leg::RightBack,   Joint::Femur), angle_sensor_right_back_femur},
    {make_key(Leg::RightBack,   Joint::Tibia), angle_sensor_right_back_tibia}
  };

  /* This map contains the angle offsets of all the sensors
   * angles as reported by the leg controllers. At initialisation
   * these are zero and are then set during the calibration
   * state.
   **/
  bool is_angle_position_sensor_offset_calibration_complete = false;
  std::map<LegJointKey, float> angle_position_sensor_offset_map;
  for (auto [leg, joint] : HYDRAULIC_LEG_JOINT_LIST)
    angle_position_sensor_offset_map[make_key(leg, joint)] = 0.0f;

  std::map<Leg, common::sensor::interface::SharedBumperSensor> bumper_sensor_map =
  {
    {Leg::LeftFront,   tibia_tip_bumper_left_front},
    {Leg::LeftMiddle,  tibia_tip_bumper_left_middle},
    {Leg::LeftBack,    tibia_tip_bumper_left_back},
    {Leg::RightFront,  tibia_tip_bumper_right_front},
    {Leg::RightMiddle, tibia_tip_bumper_right_middle},
    {Leg::RightBack,   tibia_tip_bumper_right_back}
  };

  auto isGaitControllerInputDataValid = [](std::map<LegJointKey, common::sensor::interface::SharedAnglePositionSensor> const & ap_map,
                                           std::map<Leg, common::sensor::interface::SharedBumperSensor> const & b_map) -> bool
  {
    for (auto [leg, joint] : LEG_JOINT_LIST)
      if (!ap_map.at(make_key(leg, joint))->get().has_value())
        return false;

    for (auto leg : LEG_LIST)
      if (!b_map.at(leg)->get().has_value())
        return false;

    return true;
  };

  auto toStr = [](std::map<LegJointKey, common::sensor::interface::SharedAnglePositionSensor> const & angle_position,
                  std::map<Leg, common::sensor::interface::SharedBumperSensor> const & bumper) -> std::string
  {
    std::stringstream msg;

    msg << "\n"
        << "Left\n"
        << "  Front :"
        << "  Coxa: "   << angle_position.at(make_key(Leg::LeftFront, Joint::Coxa))->toStr()
        << "  Femur: "  << angle_position.at(make_key(Leg::LeftFront, Joint::Femur))->toStr()
        << "  Tibia: "  << angle_position.at(make_key(Leg::LeftFront, Joint::Tibia))->toStr()
        << "  Tip: "    << bumper.at(Leg::LeftFront)->toStr()
        << "\n"
        << "  Middle:"
        << "  Coxa: "   << angle_position.at(make_key(Leg::LeftMiddle, Joint::Coxa))->toStr()
        << "  Femur: "  << angle_position.at(make_key(Leg::LeftMiddle, Joint::Femur))->toStr()
        << "  Tibia: "  << angle_position.at(make_key(Leg::LeftMiddle, Joint::Tibia))->toStr()
        << "  Tip: "    << bumper.at(Leg::LeftMiddle)->toStr()
        << "\n"
        << "  Back  :"
        << "  Coxa: "   << angle_position.at(make_key(Leg::LeftBack, Joint::Coxa))->toStr()
        << "  Femur: "  << angle_position.at(make_key(Leg::LeftBack, Joint::Femur))->toStr()
        << "  Tibia: "  << angle_position.at(make_key(Leg::LeftBack, Joint::Tibia))->toStr()
        << "  Tip: "    << bumper.at(Leg::LeftBack)->toStr()
        << "\n"
        << "Right\n"
        << "  Front :"
        << "  Coxa: "   << angle_position.at(make_key(Leg::RightFront, Joint::Coxa))->toStr()
        << "  Femur: "  << angle_position.at(make_key(Leg::RightFront, Joint::Femur))->toStr()
        << "  Tibia: "  << angle_position.at(make_key(Leg::RightFront, Joint::Tibia))->toStr()
        << "  Tip: "    << bumper.at(Leg::RightFront)->toStr()
        << "\n"
        << "  Middle:"
        << "  Coxa: "   << angle_position.at(make_key(Leg::RightMiddle, Joint::Coxa))->toStr()
        << "  Femur: "  << angle_position.at(make_key(Leg::RightMiddle, Joint::Femur))->toStr()
        << "  Tibia: "  << angle_position.at(make_key(Leg::RightMiddle, Joint::Tibia))->toStr()
        << "  Tip: "    << bumper.at(Leg::RightMiddle)->toStr()
        << "\n"
        << "  Back  :"
        << "  Coxa: "   << angle_position.at(make_key(Leg::RightBack, Joint::Coxa))->toStr()
        << "  Femur: "  << angle_position.at(make_key(Leg::RightBack, Joint::Femur))->toStr()
        << "  Tibia: "  << angle_position.at(make_key(Leg::RightBack, Joint::Tibia))->toStr()
        << "  Tip: "    << bumper.at(Leg::RightBack)->toStr();

    return msg.str();
  };

  /**************************************************************************************
   * STATE
   **************************************************************************************/

  TeleopCommandData teleop_cmd_data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  ros::Subscriber cmd_vel_sub = node_hdl.subscribe<geometry_msgs::Twist>("/l3xz/cmd_vel", 10, std::bind(cmd_vel_callback, std::placeholders::_1, std::ref(teleop_cmd_data)));

  gait::Controller gait_ctrl(ssc32_ctrl, orel20_ctrl, angle_position_sensor_offset_map, is_angle_position_sensor_offset_calibration_complete);
  gait::ControllerOutput prev_gait_ctrl_output(INITIAL_COXA_ANGLE_DEG,
                                               INITIAL_FEMUR_ANGLE_DEG,
                                               INITIAL_TIBIA_ANGLE_DEG,
                                               INITIAL_COXA_ANGLE_DEG,
                                               INITIAL_FEMUR_ANGLE_DEG,
                                               INITIAL_TIBIA_ANGLE_DEG,
                                               INITIAL_COXA_ANGLE_DEG,
                                               INITIAL_FEMUR_ANGLE_DEG,
                                               INITIAL_TIBIA_ANGLE_DEG,
                                               INITIAL_COXA_ANGLE_DEG,
                                               INITIAL_FEMUR_ANGLE_DEG,
                                               INITIAL_TIBIA_ANGLE_DEG,
                                               INITIAL_COXA_ANGLE_DEG,
                                               INITIAL_FEMUR_ANGLE_DEG,
                                               INITIAL_TIBIA_ANGLE_DEG,
                                               INITIAL_COXA_ANGLE_DEG,
                                               INITIAL_FEMUR_ANGLE_DEG,
                                               INITIAL_TIBIA_ANGLE_DEG);

  head::Controller head_ctrl;
  head::ControllerOutput prev_head_ctrl_output(INITIAL_PAN_ANGLE_DEG, INITIAL_TILT_ANGLE_DEG);

  /**************************************************************************************
   * MAIN LOOP
   **************************************************************************************/

  for (ros::Rate loop_rate(20);
       ros::ok();
       loop_rate.sleep())
  {
    auto const start = std::chrono::high_resolution_clock::now();

    /**************************************************************************************
     * READ FROM PERIPHERALS
     **************************************************************************************/

    dynamixel_angle_position_sensor_bulk_reader.doBulkRead();
    open_cyphal_angle_position_sensor_bulk_reader.doBulkRead();
    open_cyphal_bumper_sensor_bulk_reader.doBulkRead();

    /* Perform the correction of the sensor values from
     * the offset sensor map.
     */
    if (is_angle_position_sensor_offset_calibration_complete)
    {
      for (auto [leg, joint] : HYDRAULIC_LEG_JOINT_LIST)
      {
        glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensor * angle_pos_sensor_ptr =
          reinterpret_cast<glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensor *>(angle_position_sensor_map.at(make_key(leg, joint)).get());

        if (angle_pos_sensor_ptr->get().has_value())
        {
          float const raw_angle_deg    = angle_pos_sensor_ptr->get().value();
          float const offset_angle_deg = angle_position_sensor_offset_map.at(make_key(leg, joint));

          float const offset_corrected_angle_deg = (raw_angle_deg - offset_angle_deg);

          angle_pos_sensor_ptr->update(offset_corrected_angle_deg);
        }
      }
    }

    /**************************************************************************************
     * GAIT CONTROL
     **************************************************************************************/

    auto next_gait_ctrl_output = prev_gait_ctrl_output;

    ROS_INFO("IN: %s", toStr(angle_position_sensor_map, bumper_sensor_map).c_str());

    if (!isGaitControllerInputDataValid(angle_position_sensor_map, bumper_sensor_map))
      ROS_ERROR("gait_ctrl.update: invalid input data.");
    else
    {
      gait::ControllerInput const gait_ctrl_input(teleop_cmd_data, angle_position_sensor_map, bumper_sensor_map);
      next_gait_ctrl_output = gait_ctrl.update(gait_ctrl_input, prev_gait_ctrl_output);

      ROS_INFO("OUT: %s", next_gait_ctrl_output.toStr().c_str());

      /* Check if we need to turn on the pump. */
      if (is_angle_position_sensor_offset_calibration_complete)
      {
        unsigned int num_joints_actively_controlled = 0;
        for (auto [leg, joint] : HYDRAULIC_LEG_JOINT_LIST)
        {
          float const target_angle_deg = next_gait_ctrl_output.get_angle_deg(leg, joint);
          float const actual_angle_deg = gait_ctrl_input.get_angle_deg(leg, joint);
          float const angle_err = fabs(target_angle_deg - actual_angle_deg);
          if (angle_err > 2.0f)
            num_joints_actively_controlled++;
        }
        static uint16_t const OREL20_ESC_RPM_STEP_SIZE = 10;
        uint16_t const orel20_esc_rpm = num_joints_actively_controlled * OREL20_ESC_RPM_STEP_SIZE;
        orel20_rpm_actuator.set(orel20_esc_rpm);
      }
    }

      if (is_angle_position_sensor_offset_calibration_complete)
      {
        for (auto [leg, joint] : LEG_JOINT_LIST)
        {
          /* Write the target angles to the actual angle position actuators. */
          float const target_angle_deg = next_gait_ctrl_output.get_angle_deg(leg, joint);
          angle_position_actuator_map.at(make_key(leg, joint))->set(target_angle_deg);
        }
      }

    /* Copy previous output. */
    prev_gait_ctrl_output = next_gait_ctrl_output;

    /**************************************************************************************
     * HEAD CONTROL
     **************************************************************************************/

    head::ControllerInput head_ctrl_input(teleop_cmd_data,
                                          angle_sensor_sensor_head_pan,
                                          angle_sensor_sensor_head_tilt);

    auto next_head_ctrl_output = prev_head_ctrl_output;

    if (head_ctrl_input.isValid())
      next_head_ctrl_output = head_ctrl.update(head_ctrl_input, prev_head_ctrl_output);
    else
      ROS_ERROR("head::ControllerInput: invalid input data.");

    angle_actuator_sensor_head_pan->set (next_head_ctrl_output[head::ControllerOutput::Angle::Pan]);
    angle_actuator_sensor_head_tilt->set(next_head_ctrl_output[head::ControllerOutput::Angle::Tilt]);

    prev_head_ctrl_output = next_head_ctrl_output;

    /**************************************************************************************
     * WRITE TO PERIPHERALS
     **************************************************************************************/

    if (!dynamixel_angle_position_actuator_bulk_writer.doBulkWrite())
      ROS_ERROR("failed to set target angles for all dynamixel servos");

    ssc32_pwm_actuator_bulk_driver.doBulkWrite();
    orel20_rpm_actuator.doWrite();

    /**************************************************************************************
     * ROS
     **************************************************************************************/

    ros::spinOnce();

    /**************************************************************************************
     * LOOP RATE
     **************************************************************************************/

    auto const stop = std::chrono::high_resolution_clock::now();
    auto const duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    if (duration.count() > 50)
      ROS_WARN("main loop duration (%ld ms) exceeds limit", duration.count());
  }

  deinit_dynamixel(mx28_ctrl);
  deinit_orel20(orel20_ctrl);
  deinit_ssc32(ssc32_ctrl);

  return EXIT_SUCCESS;
}
catch (std::runtime_error const & err)
{
  ROS_ERROR("Exception (std::runtime_error) caught: %s\nTerminating ...", err.what());
  return EXIT_FAILURE;
}
catch (...)
{
  ROS_ERROR("Unhandled exception caught.\nTerminating ...");
  return EXIT_FAILURE;
}

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

bool init_dynamixel(driver::SharedMX28 & mx28_ctrl)
{
  std::optional<driver::Dynamixel::IdVect> opt_act_id_vect = mx28_ctrl->discover();

  if (!opt_act_id_vect) {
    ROS_ERROR("Zero MX-28 servos detected.");
    return false;
  }

  std::stringstream act_id_list;
  for (auto id : opt_act_id_vect.value())
    act_id_list << static_cast<int>(id) << " ";
  ROS_INFO("Detected Dynamixel MX-28: { %s}", act_id_list.str().c_str());

  bool all_req_id_found = true;
  for (auto req_id : glue::l3xz::ELROB2022::DYNAMIXEL_ID_VECT)
  {
    bool const req_id_found = std::count(opt_act_id_vect.value().begin(),
                                         opt_act_id_vect.value().end(),
                                         req_id) > 0;
    if (!req_id_found) {
      all_req_id_found = false;
      ROS_ERROR("Unable to detect required dynamixel with node id %d", static_cast<int>(req_id));
    }
  }
  if (!all_req_id_found)
    return false;

  mx28_ctrl->torqueOn(glue::l3xz::ELROB2022::DYNAMIXEL_ID_VECT);

  return true;
}

void deinit_dynamixel(driver::SharedMX28 & mx28_ctrl)
{
  mx28_ctrl->torqueOff(glue::l3xz::ELROB2022::DYNAMIXEL_ID_VECT);
}

bool init_open_cyphal(phy::opencyphal::Node & open_cyphal_node,
                      glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensorBulkReader & open_cyphal_angle_position_sensor_bulk_reader,
                      glue::l3xz::ELROB2022::OpenCyphalBumperSensorBulkReader & open_cyphal_bumper_sensor_bulk_reader)
{
  if (!open_cyphal_node.subscribe<uavcan::node::Heartbeat_1_0<>>([](CanardTransfer const & transfer)
  {
    uavcan::node::Heartbeat_1_0<> const hb = uavcan::node::Heartbeat_1_0<>::deserialize(transfer);
    ROS_DEBUG("[%d] Heartbeat received\n\tMode = %d", transfer.remote_node_id, hb.data.mode.value);
  }))
  {
    ROS_ERROR("init_open_cyphal failed to subscribe to 'uavcan::node::Heartbeat_1_0'");
    return false;
  }


  if (!open_cyphal_node.subscribe<uavcan::primitive::scalar::Real32_1_0<1001>>([](CanardTransfer const & transfer)
  {
    uavcan::primitive::scalar::Real32_1_0<1001> const input_voltage = uavcan::primitive::scalar::Real32_1_0<1001>::deserialize(transfer);
    ROS_DEBUG("[%d] Battery Voltage = %f", transfer.remote_node_id, input_voltage.data.value);
  }))
  {
    ROS_ERROR("init_open_cyphal failed to subscribe to 'uavcan::primitive::scalar::Real32_1_0<1001>'");
    return false;
  }


  if (!open_cyphal_node.subscribe<uavcan::primitive::scalar::Real32_1_0<1002>>([&open_cyphal_angle_position_sensor_bulk_reader](CanardTransfer const & transfer)
  {
    uavcan::primitive::scalar::Real32_1_0<1002> const as5048_a_angle = uavcan::primitive::scalar::Real32_1_0<1002>::deserialize(transfer);
    open_cyphal_angle_position_sensor_bulk_reader.update_femur_angle(transfer.remote_node_id, as5048_a_angle.data.value);
    ROS_DEBUG("[%d] Angle[AS5048 A] = %f", transfer.remote_node_id, as5048_a_angle.data.value);
  }))
  {
    ROS_ERROR("init_open_cyphal failed to subscribe to 'uavcan::primitive::scalar::Real32_1_0<1002>'");
    return false;
  }


  if (!open_cyphal_node.subscribe<uavcan::primitive::scalar::Real32_1_0<1003>>([&open_cyphal_angle_position_sensor_bulk_reader](CanardTransfer const & transfer)
  {
    uavcan::primitive::scalar::Real32_1_0<1003> const as5048_b_angle = uavcan::primitive::scalar::Real32_1_0<1003>::deserialize(transfer);
    open_cyphal_angle_position_sensor_bulk_reader.update_tibia_angle(transfer.remote_node_id, as5048_b_angle.data.value);
    ROS_DEBUG("[%d] Angle[AS5048 B] = %f", transfer.remote_node_id, as5048_b_angle.data.value);
  }))
  {
    ROS_ERROR("init_open_cyphal failed to subscribe to 'uavcan::primitive::scalar::Real32_1_0<1003>'");
    return false;
  }


  if (!open_cyphal_node.subscribe<uavcan::primitive::scalar::Bit_1_0<1004>>([&open_cyphal_bumper_sensor_bulk_reader](CanardTransfer const & transfer)
  {
    uavcan::primitive::scalar::Bit_1_0<1004> const tibia_endpoint_switch = uavcan::primitive::scalar::Bit_1_0<1004>::deserialize(transfer);
    open_cyphal_bumper_sensor_bulk_reader.update_bumper_sensor(transfer.remote_node_id, tibia_endpoint_switch.data.value);
    ROS_DEBUG("[%d] Tibia Endpoint Switch %d", transfer.remote_node_id, tibia_endpoint_switch.data.value);
  }))
  {
    ROS_ERROR("init_open_cyphal failed to subscribe to 'uavcan::primitive::scalar::Bit_1_0<1004>'");
    return false;
  }

  return true;
}

void deinit_orel20(driver::SharedOrel20 orel20_ctrl)
{
  orel20_ctrl->setRPM(0);
  orel20_ctrl->spinOnce();
}

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

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr & msg, TeleopCommandData & teleop_cmd_data)
{
  teleop_cmd_data.linear_velocity_x           = msg->linear.x;
  teleop_cmd_data.linear_velocity_y           = msg->linear.y;
  teleop_cmd_data.angular_velocity_head_tilt  = msg->angular.x;
  teleop_cmd_data.angular_velocity_head_pan   = msg->angular.y;
  teleop_cmd_data.angular_velocity_z          = msg->angular.z;

  ROS_DEBUG("v_tilt = %.2f, v_pan = %.2f", teleop_cmd_data.angular_velocity_head_tilt, teleop_cmd_data.angular_velocity_head_pan);
}
