/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <IoNode.h>

#include <iomanip>

#include <glue/DynamixelAnglePositionSensorBulkReader.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

IoNode::IoNode(
  dynamixel::SharedMX28 mx28_ctrl,
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
)
: Node("l3xz_io")
, _mx28_ctrl{mx28_ctrl}
, _open_cyphal_angle_position_sensor_bulk_reader{open_cyphal_angle_position_sensor_bulk_reader}
, _open_cyphal_bumper_sensor_bulk_reader{open_cyphal_bumper_sensor_bulk_reader}
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
, _leg_angle_actual_msg{
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
, _head_angle_target_msg{
    []()
    {
      l3xz_head_ctrl::msg::HeadAngle msg;

      msg.pan_angle_deg  = INITIAL_PAN_ANGLE_DEG;
      msg.tilt_angle_deg = INITIAL_TILT_ANGLE_DEG;

      return msg;
    } ()
  }
{
  _timer = create_wall_timer
    (std::chrono::milliseconds(50), [this]() { this->timerCallback(); });

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
      _head_angle_target_msg = *head_angle_target_msg;
    });


  /* Open all hydraulic valves. */
  for (auto ch: SERVO_CHANNEL_LIST)
    ssc32_ctrl->setPulseWidth(ch, 2000, 50);

  /* Start the hydraulic pump. */
  orel20_ctrl->setRPM(4096);

  /* Capture the start time. */
  _start_calibration = std::chrono::high_resolution_clock::now();
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void IoNode::timerCallback()
{
  /**************************************************************************************
   * READ FROM PERIPHERALS
   **************************************************************************************/

  auto const dynamixel_angle_position_deg = glue::DynamixelAnglePositionSensorBulkReader::doBulkRead(_mx28_ctrl);

  _open_cyphal_angle_position_sensor_bulk_reader.doBulkRead();
  _open_cyphal_bumper_sensor_bulk_reader.doBulkRead();

  if (!_is_angle_position_sensor_offset_calibration_complete)
  {
    auto const now = std::chrono::high_resolution_clock::now();
    auto const duration = std::chrono::duration_cast<std::chrono::seconds>(now - _start_calibration);

    if (duration.count() > 20)
    {
      _is_angle_position_sensor_offset_calibration_complete = true;

      /* Capture the raw angles which are at this point
      * in time the raw values as reported by the sensors.
      */
      for (auto [leg, joint] : HYDRAULIC_LEG_JOINT_LIST)
        _angle_position_sensor_offset_map.at(make_key(leg, joint)) = _angle_position_sensor_map.at(make_key(leg, joint))->get().value();

      /* Print the offset values one time for
      * documentation.
      */
      std::stringstream msg;
      msg << "Left" << std::endl
          << "  Front" << std::endl
          << "    Femur: " << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::LeftFront, Joint::Femur)) << std::endl
          << "    Tibia: " << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::LeftFront, Joint::Tibia)) << std::endl
          << "  Middle" << std::endl
          << "    Femur: " << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::LeftMiddle, Joint::Femur)) << std::endl
          << "    Tibia: " << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::LeftMiddle, Joint::Tibia)) << std::endl
          << "  Back" << std::endl
          << "    Femur: " << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::LeftBack, Joint::Femur)) << std::endl
          << "    Tibia: " << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::LeftBack, Joint::Tibia)) << std::endl
          << "Right" << std::endl
          << "  Front" << std::endl
          << "    Femur: " << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::RightFront, Joint::Femur)) << std::endl
          << "    Tibia: " << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::RightFront, Joint::Tibia)) << std::endl
          << "  Middle: " << std::endl
          << "    Femur: " << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::RightMiddle, Joint::Femur)) << std::endl
          << "    Tibia: " << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::RightMiddle, Joint::Tibia)) << std::endl
          << "  Back" << std::endl
          << "    Femur: " << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::RightBack, Joint::Femur)) << std::endl
          << "    Tibia: " << std::fixed << std::setprecision(2) << std::setfill(' ') << std::setw(6) << _angle_position_sensor_offset_map.at(make_key(Leg::RightBack, Joint::Tibia)) << std::endl
          ;

      printf("[INFO] Calibrate::update: captured offset angles ...\n%s", msg.str().c_str());
    }
  }

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

  if (_is_angle_position_sensor_offset_calibration_complete)
  {
    unsigned int num_joints_actively_controlled = 0;
    for (auto [leg, joint] : LEG_JOINT_LIST)
    {
      float const target_angle_deg = get_angle_deg(_leg_angle_target_msg, leg, joint);
      float const actual_angle_deg = get_angle_deg(_leg_angle_actual_msg, leg, joint);
      float const angle_err = fabs(target_angle_deg - actual_angle_deg);
      if (angle_err > 2.0f)
        num_joints_actively_controlled++;
    }

    if (num_joints_actively_controlled > 0)
      _orel20_rpm_actuator.set(4096);
    else
      _orel20_rpm_actuator.set(0);
  }

  /**************************************************************************************
   * HEAD CONTROL
   **************************************************************************************/

  l3xz_head_ctrl::msg::HeadAngle head_angle_actual_msg;
  head_angle_actual_msg.pan_angle_deg  = dynamixel_angle_position_deg.at(glue::DynamixelServoName::Head_Pan);
  head_angle_actual_msg.tilt_angle_deg = dynamixel_angle_position_deg.at(glue::DynamixelServoName::Head_Tilt);
  _head_angle_pub->publish(head_angle_actual_msg);

  _angle_actuator_sensor_head_pan->set (_head_angle_target_msg.pan_angle_deg);
  _angle_actuator_sensor_head_tilt->set(_head_angle_target_msg.tilt_angle_deg);

  /**************************************************************************************
   * WRITE TO PERIPHERALS
   **************************************************************************************/

  if (!_dynamixel_angle_position_actuator_bulk_writer.doBulkWrite())
    printf("[ERROR] failed to set target angles for all dynamixel servos");

  _ssc32_pwm_actuator_bulk_driver.doBulkWrite();
  _orel20_rpm_actuator.doWrite();
}

float IoNode::get_angle_deg(l3xz_gait_ctrl::msg::LegAngle const & msg, Leg const leg, Joint const joint)
{
  std::map<LegJointKey, float> const ANGLE_POSITION_MAP =
  {
    {make_key(Leg::LeftFront,   Joint::Coxa),  msg.coxa_angle_deg [0]},
    {make_key(Leg::LeftFront,   Joint::Femur), msg.femur_angle_deg[0]},
    {make_key(Leg::LeftFront,   Joint::Tibia), msg.tibia_angle_deg[0]},

    {make_key(Leg::LeftMiddle,  Joint::Coxa),  msg.coxa_angle_deg [1]},
    {make_key(Leg::LeftMiddle,  Joint::Femur), msg.femur_angle_deg[1]},
    {make_key(Leg::LeftMiddle,  Joint::Tibia), msg.tibia_angle_deg[1]},

    {make_key(Leg::LeftBack,    Joint::Coxa),  msg.coxa_angle_deg [2]},
    {make_key(Leg::LeftBack,    Joint::Femur), msg.femur_angle_deg[2]},
    {make_key(Leg::LeftBack,    Joint::Tibia), msg.tibia_angle_deg[2]},

    {make_key(Leg::RightFront,  Joint::Coxa),  msg.coxa_angle_deg [3]},
    {make_key(Leg::RightFront,  Joint::Femur), msg.femur_angle_deg[3]},
    {make_key(Leg::RightFront,  Joint::Tibia), msg.tibia_angle_deg[3]},

    {make_key(Leg::RightMiddle, Joint::Coxa),  msg.coxa_angle_deg [4]},
    {make_key(Leg::RightMiddle, Joint::Femur), msg.femur_angle_deg[4]},
    {make_key(Leg::RightMiddle, Joint::Tibia), msg.tibia_angle_deg[4]},

    {make_key(Leg::RightBack,   Joint::Coxa),  msg.coxa_angle_deg [5]},
    {make_key(Leg::RightBack,   Joint::Femur), msg.femur_angle_deg[5]},
    {make_key(Leg::RightBack,   Joint::Tibia), msg.tibia_angle_deg[5]}
  };

  return ANGLE_POSITION_MAP.at(make_key(leg, joint));
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
