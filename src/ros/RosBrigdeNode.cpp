/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <ros/RosBrigdeNode.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

RosBridgeNode::RosBridgeNode(
  driver::SharedOrel20 orel20_ctrl,
  driver::SharedSSC32 ssc32_ctrl,
  glue::l3xz::ELROB2022::DynamixelAnglePositionSensorBulkReader & dynamixel_angle_position_sensor_bulk_reader,
  glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensorBulkReader & open_cyphal_angle_position_sensor_bulk_reader,
  glue::l3xz::ELROB2022::OpenCyphalBumperSensorBulkReader & open_cyphal_bumper_sensor_bulk_reader,
  glue::l3xz::ELROB2022::OpenCyphalLEDActuator & open_cyphal_led_actuator,
  glue::l3xz::ELROB2022::Orel20RPMActuator & orel20_rpm_actuator,
  glue::l3xz::ELROB2022::SSC32PWMActuatorBulkwriter & ssc32_pwm_actuator_bulk_driver,
  glue::l3xz::ELROB2022::DynamixelAnglePositionActuatorBulkWriter & dynamixel_angle_position_actuator_bulk_writer,
  bool & is_angle_position_sensor_offset_calibration_complete,
  std::map<LegJointKey, common::sensor::interface::SharedAnglePositionSensor> & angle_position_sensor_map,
  std::map<LegJointKey, float> & angle_position_sensor_offset_map,
  std::map<Leg, common::sensor::interface::SharedBumperSensor> & bumper_sensor_map,
  glue::l3xz::ELROB2022::SharedDynamixelAnglePositionActuator angle_actuator_sensor_head_pan,
  glue::l3xz::ELROB2022::SharedDynamixelAnglePositionActuator angle_actuator_sensor_head_tilt,
  std::map<LegJointKey, common::actuator::interface::SharedAnglePositionActuator> & angle_position_actuator_map,
  glue::l3xz::ELROB2022::SharedDynamixelAnglePositionSensor angle_sensor_sensor_head_pan,
  glue::l3xz::ELROB2022::SharedDynamixelAnglePositionSensor angle_sensor_sensor_head_tilt
)
: Node("l3xz_io")
, _dynamixel_angle_position_sensor_bulk_reader{dynamixel_angle_position_sensor_bulk_reader}
, _open_cyphal_angle_position_sensor_bulk_reader{open_cyphal_angle_position_sensor_bulk_reader}
, _open_cyphal_bumper_sensor_bulk_reader{open_cyphal_bumper_sensor_bulk_reader}
, _open_cyphal_led_actuator{open_cyphal_led_actuator}
, _orel20_rpm_actuator{orel20_rpm_actuator}
, _ssc32_pwm_actuator_bulk_driver{ssc32_pwm_actuator_bulk_driver}
, _dynamixel_angle_position_actuator_bulk_writer{dynamixel_angle_position_actuator_bulk_writer}
, _is_angle_position_sensor_offset_calibration_complete{is_angle_position_sensor_offset_calibration_complete}
, _angle_position_sensor_map{angle_position_sensor_map}
, _angle_position_sensor_offset_map{angle_position_sensor_offset_map}
, _bumper_sensor_map{bumper_sensor_map}
, _angle_actuator_sensor_head_pan{angle_actuator_sensor_head_pan}
, _angle_actuator_sensor_head_tilt{angle_actuator_sensor_head_tilt}
, _angle_position_actuator_map{angle_position_actuator_map}
, _angle_sensor_sensor_head_pan{angle_sensor_sensor_head_pan}
, _angle_sensor_sensor_head_tilt{angle_sensor_sensor_head_tilt}
, _gait_ctrl{ssc32_ctrl, orel20_ctrl, angle_position_sensor_offset_map, is_angle_position_sensor_offset_calibration_complete}
, _prev_gait_ctrl_output{INITIAL_COXA_ANGLE_DEG, INITIAL_FEMUR_ANGLE_DEG, INITIAL_TIBIA_ANGLE_DEG, INITIAL_COXA_ANGLE_DEG, INITIAL_FEMUR_ANGLE_DEG, INITIAL_TIBIA_ANGLE_DEG, INITIAL_COXA_ANGLE_DEG, INITIAL_FEMUR_ANGLE_DEG, INITIAL_TIBIA_ANGLE_DEG, INITIAL_COXA_ANGLE_DEG, INITIAL_FEMUR_ANGLE_DEG, INITIAL_TIBIA_ANGLE_DEG, INITIAL_COXA_ANGLE_DEG, INITIAL_FEMUR_ANGLE_DEG, INITIAL_TIBIA_ANGLE_DEG, INITIAL_COXA_ANGLE_DEG, INITIAL_FEMUR_ANGLE_DEG, INITIAL_TIBIA_ANGLE_DEG}
, _teleop_cmd_data{0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
, _leg_angle_target_msg{
    []()
    {
      l3xz_gait_ctrl::msg::LegAngle msg;
      
      for (size_t l = 0; l < 6; l++)
      {
        msg.coxa_angle_deg [l] = INITIAL_COXA_ANGLE_DEG;
        msg.femur_angle_deg[l] = INITIAL_FEMUR_ANGLE_DEG;
        msg.tibia_angle_deg[l] = INITIAL_TIBIA_ANGLE_DEG;
      }

      return msg;
    } ()
  }
{
  _timer = create_wall_timer
    (std::chrono::milliseconds(50), [this]() { this->timerCallback(); });

  _radiation_pub = create_publisher<std_msgs::msg::Int16>("/l3xz/radiation_tick_cnt", 25);

  _cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>
    ("/l3xz/cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr const msg) { this->onCmdVelUpdate(msg); });

  _leg_angle_pub = create_publisher<l3xz_gait_ctrl::msg::LegAngle>
    ("/l3xz/ctrl/gait/angle/actual", 10);

  _leg_angle_sub = create_subscription<l3xz_gait_ctrl::msg::LegAngle>
    ("/l3xz/ctrl/gait/angle/actual", 10, [this](l3xz_gait_ctrl::msg::LegAngle::SharedPtr const leg_angle_target_msg)
    {
      _leg_angle_target_msg = *leg_angle_target_msg;
    });

  _head_angle_pub = create_publisher<l3xz_head_ctrl::msg::HeadAngle>
    ("/l3xz/ctrl/head/angle/actual", 10);

  _head_angle_sub = create_subscription<l3xz_head_ctrl::msg::HeadAngle>
    ("/l3xz/ctrl/head/angle/target", 10, [this](l3xz_head_ctrl::msg::HeadAngle::SharedPtr const head_angle_target_msg)
    {
      _angle_actuator_sensor_head_pan->set (head_angle_target_msg->pan_angle_deg);
      _angle_actuator_sensor_head_tilt->set(head_angle_target_msg->tilt_angle_deg);
    });
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void RosBridgeNode::publish_radiation_tick_count(int16_t const radiation_tick_cnt)
{
  std_msgs::msg::Int16 radiation_tick_msg;
  radiation_tick_msg.data = radiation_tick_cnt;
  _radiation_pub->publish(radiation_tick_msg);
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void RosBridgeNode::onCmdVelUpdate(geometry_msgs::msg::Twist::SharedPtr const msg)
{
  _teleop_cmd_data.linear_velocity_x           = msg->linear.x;
  _teleop_cmd_data.linear_velocity_y           = msg->linear.y;
  _teleop_cmd_data.angular_velocity_head_tilt  = msg->angular.x;
  _teleop_cmd_data.angular_velocity_head_pan   = msg->angular.y;
  _teleop_cmd_data.angular_velocity_z          = msg->angular.z;
}

void RosBridgeNode::timerCallback()
{
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
   * READ FROM PERIPHERALS
   **************************************************************************************/

  _dynamixel_angle_position_sensor_bulk_reader.doBulkRead();
  _open_cyphal_angle_position_sensor_bulk_reader.doBulkRead();
  _open_cyphal_bumper_sensor_bulk_reader.doBulkRead();

  /* Perform the correction of the sensor values from
   * the offset sensor map.
   */
  if (_is_angle_position_sensor_offset_calibration_complete)
  {
    for (auto [leg, joint] : HYDRAULIC_LEG_JOINT_LIST)
    {
      glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensor * angle_pos_sensor_ptr =
        reinterpret_cast<glue::l3xz::ELROB2022::OpenCyphalAnglePositionSensor *>(_angle_position_sensor_map.at(make_key(leg, joint)).get());

      if (angle_pos_sensor_ptr->get().has_value())
      {
        float const raw_angle_deg    = angle_pos_sensor_ptr->get().value();
        float const offset_angle_deg = _angle_position_sensor_offset_map.at(make_key(leg, joint));

        float const offset_corrected_angle_deg = (raw_angle_deg - offset_angle_deg);

        angle_pos_sensor_ptr->update(offset_corrected_angle_deg);
      }
    }
  }

  /**************************************************************************************
   * GAIT CONTROL
   **************************************************************************************/

  auto next_gait_ctrl_output = _prev_gait_ctrl_output;

  printf("[INFO] IN: %s", toStr(_angle_position_sensor_map, _bumper_sensor_map).c_str());

  if (!isGaitControllerInputDataValid(_angle_position_sensor_map, _bumper_sensor_map))
    printf("[ERROR] gait_ctrl.update: invalid input data.");
  else
  {
    gait::ControllerInput const gait_ctrl_input(_teleop_cmd_data, _angle_position_sensor_map, _bumper_sensor_map);
    next_gait_ctrl_output = _gait_ctrl.update(gait_ctrl_input, _prev_gait_ctrl_output);

    printf("[INFO] OUT: %s", next_gait_ctrl_output.toStr().c_str());

    /* Check if we need to turn on the pump. */
    if (_is_angle_position_sensor_offset_calibration_complete)
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
      if (num_joints_actively_controlled > 0)
        _orel20_rpm_actuator.set(4096);
      else
        _orel20_rpm_actuator.set(0);
    }
  }

  if (_is_angle_position_sensor_offset_calibration_complete)
  {
    for (auto [leg, joint] : LEG_JOINT_LIST)
    {
      /* Write the target angles to the actual angle position actuators. */
      float const target_angle_deg = next_gait_ctrl_output.get_angle_deg(leg, joint);
      _angle_position_actuator_map.at(make_key(leg, joint))->set(target_angle_deg);
    }
  }

  /* Copy previous output. */
  _prev_gait_ctrl_output = next_gait_ctrl_output;

  if (_is_angle_position_sensor_offset_calibration_complete)
    _open_cyphal_led_actuator.setBlinkMode(glue::l3xz::ELROB2022::OpenCyphalLEDActuator::BlinkMode::Amber);

  /**************************************************************************************
   * HEAD CONTROL
   **************************************************************************************/

  l3xz_head_ctrl::msg::HeadAngle head_angle_actual_msg;
  head_angle_actual_msg.pan_angle_deg  = _angle_sensor_sensor_head_pan->get().value();
  head_angle_actual_msg.tilt_angle_deg = _angle_sensor_sensor_head_tilt->get().value();
  _head_angle_pub->publish(head_angle_actual_msg);

  /**************************************************************************************
   * WRITE TO PERIPHERALS
   **************************************************************************************/

  if (!_dynamixel_angle_position_actuator_bulk_writer.doBulkWrite())
    printf("[ERROR] failed to set target angles for all dynamixel servos");

  _ssc32_pwm_actuator_bulk_driver.doBulkWrite();
  _orel20_rpm_actuator.doWrite();
  _open_cyphal_led_actuator.doBulkWrite();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
