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

#include <Const.h>

#include <gait/GaitController.h>
#include <head/HeadController.h>

#include <driver/ssc32/SSC32.h>
#include <driver/dynamixel/MX28.h>
#include <driver/dynamixel/Dynamixel.h>

#include <glue/l3xz/ELROB2022/Const.h>
#include <glue/l3xz/ELROB2022/SSC32PWMActuator.h>
#include <glue/l3xz/ELROB2022/SSC32PWMActuatorBulkwriter.h>
#include <glue/l3xz/ELROB2022/SSC32ValveActuator.h>
#include <glue/l3xz/ELROB2022/DynamixelAnglePositionSensor.h>
#include <glue/l3xz/ELROB2022/DynamixelAnglePositionSensorBulkReader.h>
#include <glue/l3xz/ELROB2022/DynamixelAnglePositionActuator.h>
#include <glue/l3xz/ELROB2022/DynamixelAnglePositionActuatorBulkWriter.h>

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

bool init_dynamixel  (driver::SharedMX28 & mx28_ctrl);
void deinit_dynamixel(driver::SharedMX28 & mx28_ctrl);

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr & msg, TeleopCommandData & teleop_cmd_data);

/**************************************************************************************
 * CONSTANT
 **************************************************************************************/

static std::string const DYNAMIXEL_DEVICE_NAME = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4NNZ55-if00-port0";
static float       const DYNAMIXEL_PROTOCOL_VERSION = 2.0f;
static int         const DYNAMIXEL_BAUD_RATE = 115200;

static std::string const SSC32_DEVICE_NAME = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH05FOBL-if00-port0";
static size_t      const SSC32_BAUDRATE = 115200;

static std::string const ZUBAX_BABEL_OREL20_DEVICE_NAME = "/dev/serial/by-id/usb-Zubax_Robotics_Zubax_Babel_26003A000B5150423339302000000000-if00";

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

  auto angle_sensor_coxa_leg_front_left   = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionSensor>("LEG F/L Coxa");
  auto angle_sensor_coxa_leg_front_right  = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionSensor>("LEG F/R Coxa");
  auto angle_sensor_coxa_leg_middle_left  = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionSensor>("LEG M/L Coxa");
  auto angle_sensor_coxa_leg_middle_right = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionSensor>("LEG M/R Coxa");
  auto angle_sensor_coxa_leg_back_left    = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionSensor>("LEG B/L Coxa");
  auto angle_sensor_coxa_leg_back_right   = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionSensor>("LEG B/R Coxa");
  auto angle_sensor_sensor_head_pan       = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionSensor>("HEAD Pan    ");
  auto angle_sensor_sensor_head_tilt      = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionSensor>("HEAD Tilt   ");

  glue::l3xz::ELROB2022::DynamixelAnglePositionSensorBulkReader dynamixel_angle_position_sensor_bulk_reader
  (
    mx28_ctrl,
    angle_sensor_coxa_leg_front_left,
    angle_sensor_coxa_leg_front_right,
    angle_sensor_coxa_leg_middle_left,
    angle_sensor_coxa_leg_middle_right,
    angle_sensor_coxa_leg_back_left,
    angle_sensor_coxa_leg_back_right,
    angle_sensor_sensor_head_pan,
    angle_sensor_sensor_head_tilt
  );

  glue::l3xz::ELROB2022::DynamixelAnglePositionActuatorBulkWriter dynamixel_angle_position_actuator_bulk_writer(mx28_ctrl);

  auto angle_actuator_coxa_leg_front_left   = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("LEG F/L Coxa", 1, [&dynamixel_angle_position_actuator_bulk_writer](driver::Dynamixel::Id const id, float const angle_deg) { dynamixel_angle_position_actuator_bulk_writer.update(id, angle_deg); }, 180.0f);
  auto angle_actuator_coxa_leg_front_right  = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("LEG F/R Coxa", 2, [&dynamixel_angle_position_actuator_bulk_writer](driver::Dynamixel::Id const id, float const angle_deg) { dynamixel_angle_position_actuator_bulk_writer.update(id, angle_deg); }, 180.0f);
  auto angle_actuator_coxa_leg_middle_left  = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("LEG M/L Coxa", 3, [&dynamixel_angle_position_actuator_bulk_writer](driver::Dynamixel::Id const id, float const angle_deg) { dynamixel_angle_position_actuator_bulk_writer.update(id, angle_deg); }, 180.0f);
  auto angle_actuator_coxa_leg_middle_right = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("LEG M/R Coxa", 4, [&dynamixel_angle_position_actuator_bulk_writer](driver::Dynamixel::Id const id, float const angle_deg) { dynamixel_angle_position_actuator_bulk_writer.update(id, angle_deg); }, 180.0f);
  auto angle_actuator_coxa_leg_back_left    = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("LEG B/L Coxa", 5, [&dynamixel_angle_position_actuator_bulk_writer](driver::Dynamixel::Id const id, float const angle_deg) { dynamixel_angle_position_actuator_bulk_writer.update(id, angle_deg); }, 180.0f);
  auto angle_actuator_coxa_leg_back_right   = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("LEG B/R Coxa", 6, [&dynamixel_angle_position_actuator_bulk_writer](driver::Dynamixel::Id const id, float const angle_deg) { dynamixel_angle_position_actuator_bulk_writer.update(id, angle_deg); }, 180.0f);
  auto angle_actuator_sensor_head_pan       = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("HEAD Pan    ", 7, [&dynamixel_angle_position_actuator_bulk_writer](driver::Dynamixel::Id const id, float const angle_deg) { dynamixel_angle_position_actuator_bulk_writer.update(id, angle_deg); }, 180.0f);
  auto angle_actuator_sensor_head_tilt      = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("HEAD Tilt   ", 8, [&dynamixel_angle_position_actuator_bulk_writer](driver::Dynamixel::Id const id, float const angle_deg) { dynamixel_angle_position_actuator_bulk_writer.update(id, angle_deg); }, 180.0f);

  /**************************************************************************************
   * SSC32
   **************************************************************************************/

  auto ssc32_ctrl = std::make_shared<driver::SSC32>(SSC32_DEVICE_NAME, SSC32_BAUDRATE);

  glue::l3xz::ELROB2022::SSC32PWMActuatorBulkwriter ssc32_pwm_actuator_bulk_driver(ssc32_ctrl);

  auto pwm_actuator_valve_front_left_femur   = std::make_shared<glue::l3xz::ELROB2022::SSC32PWMActuator>("LEG F/L Femur",  0, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  auto pwm_actuator_valve_front_left_tibia   = std::make_shared<glue::l3xz::ELROB2022::SSC32PWMActuator>("LEG F/L Tibia",  1, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  auto pwm_actuator_valve_middle_left_femur  = std::make_shared<glue::l3xz::ELROB2022::SSC32PWMActuator>("LEG M/L Femur",  2, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  auto pwm_actuator_valve_middle_left_tibia  = std::make_shared<glue::l3xz::ELROB2022::SSC32PWMActuator>("LEG M/L Tibia",  3, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  auto pwm_actuator_valve_back_left_femur    = std::make_shared<glue::l3xz::ELROB2022::SSC32PWMActuator>("LEG B/L Femur",  4, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  auto pwm_actuator_valve_back_left_tibia    = std::make_shared<glue::l3xz::ELROB2022::SSC32PWMActuator>("LEG B/L Tibia",  5, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });

  auto pwm_actuator_valve_front_right_femur  = std::make_shared<glue::l3xz::ELROB2022::SSC32PWMActuator>("LEG F/R Femur", 15, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  auto pwm_actuator_valve_front_right_tibia  = std::make_shared<glue::l3xz::ELROB2022::SSC32PWMActuator>("LEG F/R Tibia", 16, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  auto pwm_actuator_valve_middle_right_femur = std::make_shared<glue::l3xz::ELROB2022::SSC32PWMActuator>("LEG M/R Femur", 17, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  auto pwm_actuator_valve_middle_right_tibia = std::make_shared<glue::l3xz::ELROB2022::SSC32PWMActuator>("LEG M/R Tibia", 18, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  auto pwm_actuator_valve_back_right_femur   = std::make_shared<glue::l3xz::ELROB2022::SSC32PWMActuator>("LEG B/R Femur", 19, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });
  auto pwm_actuator_valve_back_right_tibia   = std::make_shared<glue::l3xz::ELROB2022::SSC32PWMActuator>("LEG B/R Tibia", 20, [&ssc32_pwm_actuator_bulk_driver](uint8_t const channel, uint16_t const pulse_width_us) { ssc32_pwm_actuator_bulk_driver.update(channel, pulse_width_us); });


  auto valve_actuator_front_left_femur       = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG F/L Femur", pwm_actuator_valve_front_left_femur,   0.0);
  auto valve_actuator_front_left_tibia       = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG F/L Tibia", pwm_actuator_valve_front_left_tibia,   0.0);
  auto valve_actuator_middle_left_femur      = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG M/L Femur", pwm_actuator_valve_middle_left_femur,  0.0);
  auto valve_actuator_middle_left_tibia      = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG M/L Tibia", pwm_actuator_valve_middle_left_tibia,  0.0);
  auto valve_actuator_back_left_femur        = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG B/L Femur", pwm_actuator_valve_back_left_femur,    0.0);
  auto valve_actuator_back_left_tibia        = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG B/L Tibia", pwm_actuator_valve_back_left_tibia,    0.0);

  auto valve_actuator_front_right_femur      = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG F/R Femur", pwm_actuator_valve_front_right_femur,  0.0);
  auto valve_actuator_front_right_tibia      = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG F/R Tibia", pwm_actuator_valve_front_right_tibia,  0.0);
  auto valve_actuator_middle_right_femur     = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG M/R Femur", pwm_actuator_valve_middle_right_femur, 0.0);
  auto valve_actuator_middle_right_tibia     = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG M/R Tibia", pwm_actuator_valve_middle_right_tibia, 0.0);
  auto valve_actuator_back_right_femur       = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG B/R Femur", pwm_actuator_valve_back_right_femur,   0.0);
  auto valve_actuator_back_right_tibia       = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG B/R Tibia", pwm_actuator_valve_back_right_tibia,   0.0);

  /**************************************************************************************
   * STATE
   **************************************************************************************/

  TeleopCommandData teleop_cmd_data;
  ros::Subscriber cmd_vel_sub = node_hdl.subscribe<geometry_msgs::Twist>("/l3xz/cmd_vel", 10, std::bind(cmd_vel_callback, std::placeholders::_1, std::ref(teleop_cmd_data)));

  gait::GaitController gait_ctrl;
  gait::GaitControllerOutput gait_ctrl_output(angle_actuator_coxa_leg_front_left,
                                              angle_actuator_coxa_leg_front_right,
                                              angle_actuator_coxa_leg_middle_left,
                                              angle_actuator_coxa_leg_middle_right,
                                              angle_actuator_coxa_leg_back_left,
                                              angle_actuator_coxa_leg_back_right);


  head::HeadController head_ctrl;
  head::HeadControllerOutput head_ctrl_output(angle_actuator_sensor_head_pan,
                                              angle_actuator_sensor_head_tilt);

  /**************************************************************************************
   * MAIN LOOP
   **************************************************************************************/

  for (ros::Rate loop_rate(50);
       ros::ok();
       loop_rate.sleep())
  {
    auto const start = std::chrono::high_resolution_clock::now();

    /**************************************************************************************
     * READ FROM PERIPHERALS
     **************************************************************************************/

    /* Simultaneously read the current angle from all dynamixel servos and update the angle position sensors. */
    dynamixel_angle_position_sensor_bulk_reader.doBulkRead();

    ROS_DEBUG("L3XZ Dynamixel Current Angles:\n  %s\n  %s\n  %s\n  %s\n  %s\n  %s\n  %s\n  %s",
      angle_sensor_coxa_leg_front_left->toStr().c_str(),
      angle_sensor_coxa_leg_front_right->toStr().c_str(),
      angle_sensor_coxa_leg_middle_left->toStr().c_str(),
      angle_sensor_coxa_leg_middle_right->toStr().c_str(),
      angle_sensor_coxa_leg_back_left->toStr().c_str(),
      angle_sensor_coxa_leg_back_right->toStr().c_str(),
      angle_sensor_sensor_head_pan->toStr().c_str(),
      angle_sensor_sensor_head_tilt->toStr().c_str());

    /**************************************************************************************
     * GAIT CONTROL
     **************************************************************************************/

    gait::GaitControllerInput gait_ctrl_input(teleop_cmd_data,
                                              angle_sensor_coxa_leg_front_left,
                                              angle_sensor_coxa_leg_front_right,
                                              angle_sensor_coxa_leg_middle_left,
                                              angle_sensor_coxa_leg_middle_right,
                                              angle_sensor_coxa_leg_back_left,
                                              angle_sensor_coxa_leg_back_right);

    gait_ctrl.update(gait_ctrl_input, gait_ctrl_output);

    /**************************************************************************************
     * HEAD CONTROL
     **************************************************************************************/

    head::HeadControllerInput head_ctrl_input(teleop_cmd_data,
                                              angle_sensor_sensor_head_pan,
                                              angle_sensor_sensor_head_tilt);

    head_ctrl.update(head_ctrl_input, head_ctrl_output);

    /**************************************************************************************
     * WRITE TO PERIPHERALS
     **************************************************************************************/

    if (!dynamixel_angle_position_actuator_bulk_writer.doBulkWrite())
      ROS_ERROR("failed to set target angles for all dynamixel servos");

    ssc32_pwm_actuator_bulk_driver.doBulkWrite();

    /**************************************************************************************
     * ROS
     **************************************************************************************/

    ros::spinOnce();

    auto const stop = std::chrono::high_resolution_clock::now();
    auto const duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    if (duration.count() > 25)
      ROS_WARN("main loop duration (%ld ms) exceeds limit", duration.count());
  }

  deinit_dynamixel(mx28_ctrl);

  return EXIT_SUCCESS;
}
catch (std::runtime_error const & err)
{
  ROS_ERROR("Exception caught: %s\nTerminating ...", err.what());
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

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr & msg, TeleopCommandData & teleop_cmd_data)
{
  teleop_cmd_data.linear_velocity_x           = msg->linear.x;
  teleop_cmd_data.linear_velocity_y           = msg->linear.y;
  teleop_cmd_data.angular_velocity_head_tilt  = msg->angular.x;
  teleop_cmd_data.angular_velocity_head_pan   = msg->angular.y;
  teleop_cmd_data.angular_velocity_z          = msg->angular.z;

  ROS_DEBUG("v_tilt = %.2f, v_pan = %.2f", teleop_cmd_data.angular_velocity_head_tilt, teleop_cmd_data.angular_velocity_head_pan);
}
