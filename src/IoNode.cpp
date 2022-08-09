/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_io/IoNode.h>

#include <iomanip>

#include <l3xz_io/const/LegJointList.h>
#include <l3xz_io/const/InitialAngle.h>
#include <l3xz_io/const/DynamixelIdList.h>

#include <l3xz_io/glue/DynamixelAnglePositionReader.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CONSTANT
 **************************************************************************************/

static std::string const DYNAMIXEL_DEVICE_NAME      = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4NNZ55-if00-port0";
static float       const DYNAMIXEL_PROTOCOL_VERSION = 2.0f;
static int         const DYNAMIXEL_BAUD_RATE        = 115200;

static std::string const SSC32_DEVICE_NAME = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AH05FOBL-if00-port0";
static size_t      const SSC32_BAUDRATE    = 115200;

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

IoNode::IoNode()
: Node("l3xz_io")
, _state{State::Init}
, _open_cyphal_can_if("can0", false)
, _open_cyphal_node(_open_cyphal_can_if)
, _dynamixel_ctrl{new dynamixel::Dynamixel(DYNAMIXEL_DEVICE_NAME, DYNAMIXEL_PROTOCOL_VERSION, DYNAMIXEL_BAUD_RATE)}
, _mx28_ctrl{new dynamixel::MX28(_dynamixel_ctrl)}
, _ssc32_ctrl{new driver::SSC32(SSC32_DEVICE_NAME, DYNAMIXEL_BAUD_RATE)}
, _hydraulic_pump{_open_cyphal_node, get_logger()}
, _bumber_sensor_reader{_open_cyphal_node, get_logger()}
, _hydraulic_angle_position_reader{_open_cyphal_node, get_logger()}
, _dynamixel_angle_position_writer{}
, _ssc32_valve_writer{}
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

IoNode::~IoNode()
{
  deinit_dynamixel();
  deinit_ssc32();
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void IoNode::timerCallback()
{
  State next_state = _state;
  switch (_state)
  {
  case State::Init:   next_state = handle_Init();   break;
  case State::Active: next_state = handle_Active(); break;
  }
  _state = next_state;
}

IoNode::State IoNode::handle_Init()
{
  if (!init_dynamixel())
    RCLCPP_ERROR(get_logger(), "failed to initialize all dynamixel servos.");

  init_ssc32();

  return State::Active;
}

IoNode::State IoNode::handle_Active()
{
  /**************************************************************************************
   * READ FROM PERIPHERALS
   **************************************************************************************/

  auto const [dynamixel_leg_joint_angle_position, dynamixel_head_joint_angle_position] = glue::DynamixelAnglePositionReader::doBulkRead(_mx28_ctrl);
  auto const hydraulic_angle_position_deg = _hydraulic_angle_position_reader.doBulkRead();
  auto const tibia_tip_bumper_map = _bumber_sensor_reader.doBulkRead();

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

  bool turn_hydraulic_pump_on = false;
  for (auto [leg, joint] : LEG_JOINT_LIST)
  {
    float const target_angle_deg = get_angle_deg(_leg_angle_target_msg, leg, joint);
    float const actual_angle_deg = get_angle_deg( leg_angle_actual_msg, leg, joint);
    float const angle_err = fabs(target_angle_deg - actual_angle_deg);
    if (angle_err > 2.0f)
      turn_hydraulic_pump_on = true;
  }

  if (turn_hydraulic_pump_on)
    _hydraulic_pump.setRPM(4096);
  else
    _hydraulic_pump.setRPM(0);

  /**************************************************************************************
   * WRITE TARGET STATE TO PERIPHERAL DRIVERS
   **************************************************************************************/

  _dynamixel_angle_position_writer.update(HeadJointKey::Pan,  _head_angle_target_msg.pan_angle_deg);
  _dynamixel_angle_position_writer.update(HeadJointKey::Tilt, _head_angle_target_msg.tilt_angle_deg);

  _dynamixel_angle_position_writer.update(make_key(Leg::LeftFront,   Joint::Coxa), _leg_angle_target_msg.coxa_angle_deg[0]);
  _dynamixel_angle_position_writer.update(make_key(Leg::LeftMiddle,  Joint::Coxa), _leg_angle_target_msg.coxa_angle_deg[1]);
  _dynamixel_angle_position_writer.update(make_key(Leg::LeftBack,    Joint::Coxa), _leg_angle_target_msg.coxa_angle_deg[2]);
  _dynamixel_angle_position_writer.update(make_key(Leg::RightBack,   Joint::Coxa), _leg_angle_target_msg.coxa_angle_deg[3]);
  _dynamixel_angle_position_writer.update(make_key(Leg::RightMiddle, Joint::Coxa), _leg_angle_target_msg.coxa_angle_deg[4]);
  _dynamixel_angle_position_writer.update(make_key(Leg::RightFront,  Joint::Coxa), _leg_angle_target_msg.coxa_angle_deg[5]);

  /**************************************************************************************
   * WRITE TO PERIPHERALS
   **************************************************************************************/

  if (!_dynamixel_angle_position_writer.doBulkWrite(_mx28_ctrl))
    RCLCPP_ERROR(get_logger(), "failed to set target angles for all dynamixel servos");

  _hydraulic_pump.doWrite();
  _ssc32_valve_writer.doBulkWrite(_ssc32_ctrl);

  return State::Active;
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

bool IoNode::init_dynamixel()
{
  std::optional<dynamixel::Dynamixel::IdVect> opt_act_id_vect = _mx28_ctrl->discover();

  if (!opt_act_id_vect) {
    RCLCPP_ERROR(get_logger(), "error, zero MX-28 servos detected.");
    return false;
  }

  std::stringstream act_id_list;
  for (auto id : opt_act_id_vect.value())
    act_id_list << static_cast<int>(id) << " ";
  RCLCPP_INFO(get_logger(), "detected Dynamixel MX-28: { %s}", act_id_list.str().c_str());

  bool all_req_id_found = true;
  for (auto req_id : DYNAMIXEL_ID_LIST)
  {
    bool const req_id_found = std::count(opt_act_id_vect.value().begin(),
                                         opt_act_id_vect.value().end(),
                                         req_id) > 0;
    if (!req_id_found) {
      all_req_id_found = false;
      RCLCPP_ERROR(get_logger(), "error, unable to detect required dynamixel with node id %d", static_cast<int>(req_id));
    }
  }
  if (!all_req_id_found)
    return false;

  _mx28_ctrl->torqueOn(DYNAMIXEL_ID_LIST);

  return true;
}

void IoNode::deinit_dynamixel()
{
  _mx28_ctrl->torqueOff(DYNAMIXEL_ID_LIST);
}

void IoNode::init_ssc32()
{
  /* Set all servos to neutral position, this
   * means that all valves are turned off.
   */
  for (auto ch = driver::SSC32::MIN_CHANNEL; ch <= driver::SSC32::MAX_CHANNEL; ch++)
    _ssc32_ctrl->setPulseWidth(ch, 1500, 50);
}

void IoNode::deinit_ssc32()
{
  init_ssc32();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
