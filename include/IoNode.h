/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef ROS_ROS_BRIDGE_NODE_H_
#define ROS_ROS_BRIDGE_NODE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <Types.h>

#include <driver/ssc32/SSC32.h>
#include <driver/orel20/Orel20.h>

#include <glue/l3xz/ELROB2022/Orel20RPMActuator.h>
#include <glue/l3xz/ELROB2022/SSC32PWMActuatorBulkwriter.h>
#include <glue/l3xz/ELROB2022/OpenCyphalBumperSensorBulkReader.h>
#include <glue/l3xz/ELROB2022/OpenCyphalAnglePositionSensorBulkReader.h>
#include <glue/l3xz/ELROB2022/DynamixelAnglePositionActuator.h>
#include <glue/l3xz/ELROB2022/DynamixelAnglePositionActuatorBulkWriter.h>

#include <l3xz_gait_ctrl/msg/leg_angle.hpp>
#include <l3xz_head_ctrl/msg/head_angle.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class IoNode : public rclcpp::Node
{
public:
  IoNode(
    driver::SharedMX28 mx28_ctrl,
    driver::SharedOrel20 orel20_ctrl,
    driver::SharedSSC32 ssc32_ctrl,
    glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensorBulkReader & open_cyphal_angle_position_sensor_bulk_reader,
    glue::l3xz::ELROB2022::OpenCyphalBumperSensorBulkReader & open_cyphal_bumper_sensor_bulk_reader,
    glue::l3xz::ELROB2022::Orel20RPMActuator & orel20_rpm_actuator,
    glue::l3xz::ELROB2022::SSC32PWMActuatorBulkwriter & ssc32_pwm_actuator_bulk_driver,
    glue::l3xz::ELROB2022::DynamixelAnglePositionActuatorBulkWriter & dynamixel_angle_position_actuator_bulk_writer,
    bool & is_angle_position_sensor_offset_calibration_complete,
    std::map<LegJointKey, common::sensor::interface::SharedAnglePositionSensor> & angle_position_sensor_map,
    std::map<LegJointKey, float> & angle_position_sensor_offset_map,
    std::map<Leg, common::sensor::interface::SharedBumperSensor> & bumper_sensor_map,
    glue::l3xz::ELROB2022::SharedDynamixelAnglePositionActuator angle_actuator_sensor_head_pan,
    glue::l3xz::ELROB2022::SharedDynamixelAnglePositionActuator angle_actuator_sensor_head_tilt,
    std::map<LegJointKey, common::actuator::interface::SharedAnglePositionActuator> & angle_position_actuator_map
  );

private:
  driver::SharedMX28 _mx28_ctrl;
  glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensorBulkReader & _open_cyphal_angle_position_sensor_bulk_reader;
  glue::l3xz::ELROB2022::OpenCyphalBumperSensorBulkReader & _open_cyphal_bumper_sensor_bulk_reader;
  glue::l3xz::ELROB2022::Orel20RPMActuator & _orel20_rpm_actuator;
  glue::l3xz::ELROB2022::SSC32PWMActuatorBulkwriter & _ssc32_pwm_actuator_bulk_driver;
  glue::l3xz::ELROB2022::DynamixelAnglePositionActuatorBulkWriter & _dynamixel_angle_position_actuator_bulk_writer;
  bool & _is_angle_position_sensor_offset_calibration_complete;
  std::map<LegJointKey, common::sensor::interface::SharedAnglePositionSensor> & _angle_position_sensor_map;
  std::map<LegJointKey, float> & _angle_position_sensor_offset_map;
  std::map<Leg, common::sensor::interface::SharedBumperSensor> & _bumper_sensor_map;
  glue::l3xz::ELROB2022::SharedDynamixelAnglePositionActuator _angle_actuator_sensor_head_pan;
  glue::l3xz::ELROB2022::SharedDynamixelAnglePositionActuator _angle_actuator_sensor_head_tilt;
  std::map<LegJointKey, common::actuator::interface::SharedAnglePositionActuator> & _angle_position_actuator_map;
  std::chrono::high_resolution_clock::time_point _start_calibration;

  rclcpp::TimerBase::SharedPtr _timer;

  rclcpp::Publisher<l3xz_gait_ctrl::msg::LegAngle>::SharedPtr _leg_angle_pub;
  rclcpp::Subscription<l3xz_gait_ctrl::msg::LegAngle>::SharedPtr _leg_angle_sub;
  l3xz_gait_ctrl::msg::LegAngle _leg_angle_actual_msg, _leg_angle_target_msg;

  rclcpp::Publisher<l3xz_head_ctrl::msg::HeadAngle>::SharedPtr _head_angle_pub;
  rclcpp::Subscription<l3xz_head_ctrl::msg::HeadAngle>::SharedPtr _head_angle_sub;
  l3xz_head_ctrl::msg::HeadAngle _head_angle_target_msg;

  void timerCallback();

  static float get_angle_deg(l3xz_gait_ctrl::msg::LegAngle const & msg, Leg const leg, Joint const joint);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* ROS_ROS_BRIDGE_NODE_H_ */
