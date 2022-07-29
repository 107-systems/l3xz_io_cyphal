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

#include <Const.h>

#include <glue/DynamixelAnglePositionReader.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

IoNode::IoNode(
  phy::opencyphal::Node & open_cyphal_node,
  dynamixel::SharedMX28 mx28_ctrl,
  driver::SharedOrel20 orel20_ctrl,
  driver::SharedSSC32 ssc32_ctrl,
  glue::l3xz::ELROB2022::OpenCyphalBumperSensorBulkReader & open_cyphal_bumper_sensor_bulk_reader,
  glue::l3xz::ELROB2022::Orel20RPMActuator & orel20_rpm_actuator,
  glue::l3xz::ELROB2022::SSC32PWMActuatorBulkwriter & ssc32_pwm_actuator_bulk_driver,
  std::map<Leg, common::sensor::interface::SharedBumperSensor> & bumper_sensor_map
)
: Node("l3xz_io")
, _mx28_ctrl{mx28_ctrl}
, _hydraulic_angle_position_reader{open_cyphal_node}
, _open_cyphal_bumper_sensor_bulk_reader{open_cyphal_bumper_sensor_bulk_reader}
, _orel20_rpm_actuator{orel20_rpm_actuator}
, _ssc32_pwm_actuator_bulk_driver{ssc32_pwm_actuator_bulk_driver}
, _dynamixel_angle_position_writer{}
, _bumper_sensor_map{bumper_sensor_map}
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
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void IoNode::timerCallback()
{
  /**************************************************************************************
   * READ FROM PERIPHERALS
   **************************************************************************************/

  auto const [dynamixel_leg_joint_angle_position, dynamixel_head_joint_angle_position] = glue::DynamixelAnglePositionReader::doBulkRead(_mx28_ctrl);
  auto const hydraulic_angle_position_deg = _hydraulic_angle_position_reader.doBulkRead();
  _open_cyphal_bumper_sensor_bulk_reader.doBulkRead();

  /**************************************************************************************
   * PUBLISH ACTUAL SYSTEM STATE
   **************************************************************************************/

  /* l3xz_head_ctrl *********************************************************************/
  l3xz_head_ctrl::msg::HeadAngle head_angle_actual_msg;
  head_angle_actual_msg.pan_angle_deg  = dynamixel_head_joint_angle_position.at(HeadJointKey::Pan);
  head_angle_actual_msg.tilt_angle_deg = dynamixel_head_joint_angle_position.at(HeadJointKey::Tilt);
  _head_angle_pub->publish(head_angle_actual_msg);

  /* l3xz_gait_ctrl *********************************************************************/
  l3xz_gait_ctrl::msg::LegAngle leg_angle_actual_msg;

  leg_angle_actual_msg.coxa_angle_deg [0] = dynamixel_leg_joint_angle_position.at(make_key(Leg::LeftFront,    Joint::Coxa));
  leg_angle_actual_msg.coxa_angle_deg [1] = dynamixel_leg_joint_angle_position.at(make_key(Leg::LeftMiddle,   Joint::Coxa));
  leg_angle_actual_msg.coxa_angle_deg [2] = dynamixel_leg_joint_angle_position.at(make_key(Leg::LeftBack,     Joint::Coxa));
  leg_angle_actual_msg.coxa_angle_deg [3] = dynamixel_leg_joint_angle_position.at(make_key(Leg::RightBack,    Joint::Coxa));
  leg_angle_actual_msg.coxa_angle_deg [4] = dynamixel_leg_joint_angle_position.at(make_key(Leg::RightMiddle,  Joint::Coxa));
  leg_angle_actual_msg.coxa_angle_deg [5] = dynamixel_leg_joint_angle_position.at(make_key(Leg::RightFront,   Joint::Coxa));

  leg_angle_actual_msg.femur_angle_deg[0] = hydraulic_angle_position_deg.at(make_key(Leg::LeftFront,   Joint::Femur));
  leg_angle_actual_msg.femur_angle_deg[1] = hydraulic_angle_position_deg.at(make_key(Leg::LeftMiddle,  Joint::Femur));
  leg_angle_actual_msg.femur_angle_deg[2] = hydraulic_angle_position_deg.at(make_key(Leg::LeftBack,    Joint::Femur));
  leg_angle_actual_msg.femur_angle_deg[3] = hydraulic_angle_position_deg.at(make_key(Leg::RightBack,   Joint::Femur));
  leg_angle_actual_msg.femur_angle_deg[4] = hydraulic_angle_position_deg.at(make_key(Leg::RightMiddle, Joint::Femur));
  leg_angle_actual_msg.femur_angle_deg[5] = hydraulic_angle_position_deg.at(make_key(Leg::RightFront,  Joint::Femur));

  leg_angle_actual_msg.tibia_angle_deg[0] = hydraulic_angle_position_deg.at(make_key(Leg::LeftFront,   Joint::Tibia));
  leg_angle_actual_msg.tibia_angle_deg[1] = hydraulic_angle_position_deg.at(make_key(Leg::LeftMiddle,  Joint::Tibia));
  leg_angle_actual_msg.tibia_angle_deg[2] = hydraulic_angle_position_deg.at(make_key(Leg::LeftBack,    Joint::Tibia));
  leg_angle_actual_msg.tibia_angle_deg[3] = hydraulic_angle_position_deg.at(make_key(Leg::RightBack,   Joint::Tibia));
  leg_angle_actual_msg.tibia_angle_deg[4] = hydraulic_angle_position_deg.at(make_key(Leg::RightMiddle, Joint::Tibia));
  leg_angle_actual_msg.tibia_angle_deg[5] = hydraulic_angle_position_deg.at(make_key(Leg::RightFront,  Joint::Tibia));

  _leg_angle_pub->publish(leg_angle_actual_msg);

  /**************************************************************************************
   * GAIT CONTROL
   **************************************************************************************/

  unsigned int num_joints_actively_controlled = 0;
  for (auto [leg, joint] : LEG_JOINT_LIST)
  {
    float const target_angle_deg = get_angle_deg(_leg_angle_target_msg, leg, joint);
    float const actual_angle_deg = get_angle_deg( leg_angle_actual_msg, leg, joint);
    float const angle_err = fabs(target_angle_deg - actual_angle_deg);
    if (angle_err > 2.0f)
      num_joints_actively_controlled++;
  }

  if (num_joints_actively_controlled > 0)
    _orel20_rpm_actuator.set(4096);
  else
    _orel20_rpm_actuator.set(0);

  /**************************************************************************************
   * WRITE TARGET STATE TO PERIPHERAL DRIVERS
   **************************************************************************************/

  _dynamixel_angle_position_writer.update(glue::DynamixelServoName::Head_Pan,  _head_angle_target_msg.pan_angle_deg);
  _dynamixel_angle_position_writer.update(glue::DynamixelServoName::Head_Tilt, _head_angle_target_msg.tilt_angle_deg);

  /**************************************************************************************
   * WRITE TO PERIPHERALS
   **************************************************************************************/

  if (!_dynamixel_angle_position_writer.doBulkWrite(_mx28_ctrl))
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
