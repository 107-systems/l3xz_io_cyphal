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

#include <Robot.h>
#include <Const.h>

#include <state/RobotStateInput.h>
#include <state/RobotStateOutput.h>

#include <driver/ssc32/SSC32.h>
#include <driver/dynamixel/MX28.h>
#include <driver/dynamixel/Dynamixel.h>

#include <glue/l3xz/ELROB2022/Const.h>
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

static std::string const DYNAMIXEL_DEVICE_NAME = "/dev/ttyUSB0";
static float       const DYNAMIXEL_PROTOCOL_VERSION = 2.0f;
static int         const DYNAMIXEL_BAUD_RATE = 115200;

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

  auto angle_actuator_coxa_leg_front_left   = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("LEG F/L Coxa", 180.0f);
  auto angle_actuator_coxa_leg_front_right  = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("LEG F/R Coxa", 180.0f);
  auto angle_actuator_coxa_leg_middle_left  = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("LEG M/L Coxa", 180.0f);
  auto angle_actuator_coxa_leg_middle_right = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("LEG M/R Coxa", 180.0f);
  auto angle_actuator_coxa_leg_back_left    = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("LEG B/L Coxa", 180.0f);
  auto angle_actuator_coxa_leg_back_right   = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("LEG B/R Coxa", 180.0f);
  auto angle_actuator_sensor_head_pan       = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("HEAD Pan    ", 180.0f);
  auto angle_actuator_sensor_head_tilt      = std::make_shared<glue::l3xz::ELROB2022::DynamixelAnglePositionActuator>("HEAD Tilt   ", 180.0f);

  glue::l3xz::ELROB2022::DynamixelAnglePositionActuatorBulkWriter dynamixel_angle_position_actuator_bulk_writer
  (
    mx28_ctrl,
    angle_actuator_coxa_leg_front_left,
    angle_actuator_coxa_leg_front_right,
    angle_actuator_coxa_leg_middle_left,
    angle_actuator_coxa_leg_middle_right,
    angle_actuator_coxa_leg_back_left,
    angle_actuator_coxa_leg_back_right,
    angle_actuator_sensor_head_pan,
    angle_actuator_sensor_head_tilt
  );

  /**************************************************************************************
   * SSC32
   **************************************************************************************/

  auto ssc32_ctrl = std::make_shared<driver::SSC32>("/dev/ttyUSB0", 115200);

  auto valve_actuator_front_left_femur  = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG F/L Femur", ssc32_ctrl, 0, 0.0);
  auto valve_actuator_front_left_tibia  = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG F/L Tibia", ssc32_ctrl, 1, 0.0);
  auto valve_actuator_middle_left_femur = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG M/L Femur", ssc32_ctrl, 2, 0.0);
  auto valve_actuator_middle_left_tibia = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG M/L Tibia", ssc32_ctrl, 3, 0.0);
  auto valve_actuator_back_left_femur   = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG B/L Femur", ssc32_ctrl, 4, 0.0);
  auto valve_actuator_back_left_tibia   = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG B/L Tibia", ssc32_ctrl, 5, 0.0);

  auto valve_actuator_front_right_femur  = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG F/R Femur", ssc32_ctrl, 15, 0.0);
  auto valve_actuator_front_right_tibia  = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG F/R Tibia", ssc32_ctrl, 16, 0.0);
  auto valve_actuator_middle_right_femur = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG M/R Femur", ssc32_ctrl, 17, 0.0);
  auto valve_actuator_middle_right_tibia = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG M/R Tibia", ssc32_ctrl, 18, 0.0);
  auto valve_actuator_back_right_femur   = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG B/R Femur", ssc32_ctrl, 19, 0.0);
  auto valve_actuator_back_right_tibia   = std::make_shared<glue::l3xz::ELROB2022::SSC32ValveActuator>("LEG B/R Tibia", ssc32_ctrl, 20, 0.0);

  /**************************************************************************************
   * STATE
   **************************************************************************************/

  TeleopCommandData teleop_cmd_data;
  ros::Subscriber cmd_vel_sub = node_hdl.subscribe<geometry_msgs::Twist>("/l3xz/cmd_vel", 10, std::bind(cmd_vel_callback, std::placeholders::_1, std::ref(teleop_cmd_data)));


  RobotStateInput robot_state_input   (angle_sensor_coxa_leg_front_left,
                                       angle_sensor_coxa_leg_front_right,
                                       angle_sensor_coxa_leg_middle_left,
                                       angle_sensor_coxa_leg_middle_right,
                                       angle_sensor_coxa_leg_back_left,
                                       angle_sensor_coxa_leg_back_right);

  RobotStateOutput robot_state_output (angle_actuator_coxa_leg_front_left,
                                       angle_actuator_coxa_leg_front_right,
                                       angle_actuator_coxa_leg_middle_left,
                                       angle_actuator_coxa_leg_middle_right,
                                       angle_actuator_coxa_leg_back_left,
                                       angle_actuator_coxa_leg_back_right);

  Robot robot_ctrl;


  for (ros::Rate loop_rate(50);
       ros::ok();
       loop_rate.sleep())
  {
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

    /* Calculate new values for sensor head, both pan and tilt joint
     * based on the input provided by the teleop node.
     */
    static float const MAX_ANGLE_INCREMENT_PER_CYCLE_DEG = 10.0f;

    float const sensor_head_pan_actual = angle_sensor_sensor_head_pan->get().value();
    float const sensor_head_pan_target = sensor_head_pan_actual + (teleop_cmd_data.angular_velocity_head_pan * MAX_ANGLE_INCREMENT_PER_CYCLE_DEG);
    angle_actuator_sensor_head_pan->set(sensor_head_pan_target);

    float const sensor_head_tilt_actual = angle_sensor_sensor_head_tilt->get().value();
    float const sensor_head_tilt_target = sensor_head_tilt_actual + (teleop_cmd_data.angular_velocity_head_tilt * MAX_ANGLE_INCREMENT_PER_CYCLE_DEG);
    angle_actuator_sensor_head_tilt->set(sensor_head_tilt_target);

    //ROS_INFO("Head\n  Pan : actual = %.2f, target = %.2f\n  Tilt: actual = %.2f, target = %.2f", sensor_head_pan_actual, sensor_head_pan_target, sensor_head_tilt_actual, sensor_head_tilt_target);

    robot_ctrl.update(teleop_cmd_data, robot_state_input, robot_state_output);

    if (!dynamixel_angle_position_actuator_bulk_writer.doBulkWrite())
      ROS_ERROR("failed to set target angles for all dynamixel servos");

    ros::spinOnce();
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
