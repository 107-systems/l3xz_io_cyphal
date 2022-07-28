/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
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

#include <rclcpp/rclcpp.hpp>

#include <dynamixel_sdk.h>

#include <Types.h>
#include <Const.h>

#include <driver/ssc32/SSC32.h>
#include <driver/orel20/Orel20.h>
#include <driver/dynamixel/MX28.h>
#include <driver/dynamixel/Dynamixel.h>

#include <phy/opencyphal/Types.h>
#include <phy/opencyphal/Node.hpp>
#include <phy/opencyphal/SocketCAN.h>

#include <glue/DynamixelIdList.h>
#include <glue/l3xz/ELROB2022/SSC32PWMActuator.h>
#include <glue/l3xz/ELROB2022/SSC32PWMActuatorBulkwriter.h>
#include <glue/l3xz/ELROB2022/SSC32ValveActuator.h>
#include <glue/l3xz/ELROB2022/SSC32AnglePositionActuator.h>
#include <glue/l3xz/ELROB2022/OpenCyphalAnglePositionSensorBulkReader.h>
#include <glue/l3xz/ELROB2022/OpenCyphalBumperSensor.h>
#include <glue/l3xz/ELROB2022/OpenCyphalBumperSensorBulkReader.h>
#include <glue/l3xz/ELROB2022/Orel20RPMActuator.h>

#include <IoNode.h>

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

bool init_dynamixel  (dynamixel::SharedMX28 & mx28_ctrl);
void deinit_dynamixel(dynamixel::SharedMX28 & mx28_ctrl);

bool init_open_cyphal(phy::opencyphal::Node & open_cyphal_node,
                      glue::l3xz::ELROB2022::OpenCyphalBumperSensorBulkReader & open_cyphal_bumper_sensor_bulk_reader);

void deinit_orel20(driver::SharedOrel20 orel20_ctrl);

void init_ssc32  (driver::SharedSSC32 & ssc32_ctrl);
void deinit_ssc32(driver::SharedSSC32 & ssc32_ctrl);

/**************************************************************************************
 * CONSTANT
 **************************************************************************************/

static std::string const DYNAMIXEL_DEVICE_NAME = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4NNZ55-if00-port0";
static float       const DYNAMIXEL_PROTOCOL_VERSION = 2.0f;
static int         const DYNAMIXEL_BAUD_RATE = 115200;

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
   * DYNAMIXEL
   **************************************************************************************/

  auto dynamixel_ctrl = std::make_shared<dynamixel::Dynamixel>(DYNAMIXEL_DEVICE_NAME, DYNAMIXEL_PROTOCOL_VERSION, DYNAMIXEL_BAUD_RATE);
  auto mx28_ctrl = std::make_shared<dynamixel::MX28>(dynamixel_ctrl);

  if (!init_dynamixel(mx28_ctrl))
    printf("[ERROR] init_dynamixel failed.");
  printf("[INFO] init_dynamixel successfully completed.");

  /**************************************************************************************
   * OPENCYPHAL
   **************************************************************************************/

  phy::opencyphal::SocketCAN open_cyphal_can_if("can0", false);
  phy::opencyphal::Node open_cyphal_node(open_cyphal_can_if);

  glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensorBulkReader open_cyphal_angle_position_sensor_bulk_reader(open_cyphal_node);

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

  /**************************************************************************************
   * OREL 20 / DRONECAN
   **************************************************************************************/

  auto orel20_ctrl = std::make_shared<driver::Orel20>(open_cyphal_node, OREL20_NODE_ID);

  glue::l3xz::ELROB2022::Orel20RPMActuator orel20_rpm_actuator("Pump", orel20_ctrl);

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
   * ALL ANGLE POSITION ACTUATORS
   **************************************************************************************/

  std::map<Leg, common::sensor::interface::SharedBumperSensor> bumper_sensor_map =
  {
    {Leg::LeftFront,   tibia_tip_bumper_left_front},
    {Leg::LeftMiddle,  tibia_tip_bumper_left_middle},
    {Leg::LeftBack,    tibia_tip_bumper_left_back},
    {Leg::RightFront,  tibia_tip_bumper_right_front},
    {Leg::RightMiddle, tibia_tip_bumper_right_middle},
    {Leg::RightBack,   tibia_tip_bumper_right_back}
  };

  /**************************************************************************************
   * STATE
   **************************************************************************************/

  auto io_node = std::make_shared<l3xz::IoNode>
  (
    mx28_ctrl,
    orel20_ctrl,
    ssc32_ctrl,
    open_cyphal_angle_position_sensor_bulk_reader,
    open_cyphal_bumper_sensor_bulk_reader,
    orel20_rpm_actuator,
    ssc32_pwm_actuator_bulk_driver,
    bumper_sensor_map
  );

  if (!init_open_cyphal(open_cyphal_node,
                        open_cyphal_bumper_sensor_bulk_reader))
  {
    printf("[ERROR] init_open_cyphal failed.");
  }
  printf("[INFO] init_open_cyphal successfully completed.");

  /**************************************************************************************
   * MAIN LOOP
   **************************************************************************************/

  rclcpp::spin(io_node);
  rclcpp::shutdown();

  printf("[WARNING] STOPPING");

  deinit_dynamixel(mx28_ctrl);
  deinit_orel20(orel20_ctrl);
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

bool init_dynamixel(dynamixel::SharedMX28 & mx28_ctrl)
{
  std::optional<dynamixel::Dynamixel::IdVect> opt_act_id_vect = mx28_ctrl->discover();

  if (!opt_act_id_vect) {
    printf("[ERROR] Zero MX-28 servos detected.");
    return false;
  }

  std::stringstream act_id_list;
  for (auto id : opt_act_id_vect.value())
    act_id_list << static_cast<int>(id) << " ";
  printf("[INFO] Detected Dynamixel MX-28: { %s}", act_id_list.str().c_str());

  bool all_req_id_found = true;
  for (auto req_id : glue::DYNAMIXEL_ID_LIST)
  {
    bool const req_id_found = std::count(opt_act_id_vect.value().begin(),
                                         opt_act_id_vect.value().end(),
                                         req_id) > 0;
    if (!req_id_found) {
      all_req_id_found = false;
      printf("[ERROR] Unable to detect required dynamixel with node id %d", static_cast<int>(req_id));
    }
  }
  if (!all_req_id_found)
    return false;

  mx28_ctrl->torqueOn(glue::DYNAMIXEL_ID_LIST);

  return true;
}

void deinit_dynamixel(dynamixel::SharedMX28 & mx28_ctrl)
{
  mx28_ctrl->torqueOff(glue::DYNAMIXEL_ID_LIST);
}

bool init_open_cyphal(phy::opencyphal::Node & open_cyphal_node,
                      glue::l3xz::ELROB2022::OpenCyphalBumperSensorBulkReader & open_cyphal_bumper_sensor_bulk_reader)
{
  if (!open_cyphal_node.subscribe<uavcan::node::Heartbeat_1_0<>>([](CanardRxTransfer const & transfer)
  {
    //uavcan::node::Heartbeat_1_0<> const hb = uavcan::node::Heartbeat_1_0<>::deserialize(transfer);
    //printf("[DEBUG] [%d] Heartbeat received\n\tMode = %d", transfer.metadata.remote_node_id, hb.data.mode.value);
  }))
  {
    printf("[ERROR] init_open_cyphal failed to subscribe to 'uavcan::node::Heartbeat_1_0'");
    return false;
  }


  if (!open_cyphal_node.subscribe<uavcan::primitive::scalar::Real32_1_0<1001>>([](CanardRxTransfer const & transfer)
  {
    //uavcan::primitive::scalar::Real32_1_0<1001> const input_voltage = uavcan::primitive::scalar::Real32_1_0<1001>::deserialize(transfer);
    //printf("[DEBUG] [%d] Battery Voltage = %f", transfer.metadata.remote_node_id, input_voltage.data.value);
  }))
  {
    printf("[ERROR] init_open_cyphal failed to subscribe to 'uavcan::primitive::scalar::Real32_1_0<1001>'");
    return false;
  }


  if (!open_cyphal_node.subscribe<uavcan::primitive::scalar::Bit_1_0<1004>>([&open_cyphal_bumper_sensor_bulk_reader](CanardRxTransfer const & transfer)
  {
    uavcan::primitive::scalar::Bit_1_0<1004> const tibia_endpoint_switch = uavcan::primitive::scalar::Bit_1_0<1004>::deserialize(transfer);
    open_cyphal_bumper_sensor_bulk_reader.update_bumper_sensor(transfer.metadata.remote_node_id, tibia_endpoint_switch.data.value);
    //printf("[DEBUG] [%d] Tibia Endpoint Switch %d", transfer.metadata.remote_node_id, tibia_endpoint_switch.data.value);
  }))
  {
    printf("[ERROR] init_open_cyphal failed to subscribe to 'uavcan::primitive::scalar::Bit_1_0<1004>'");
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
