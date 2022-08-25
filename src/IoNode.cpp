/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCcdLUDES
 **************************************************************************************/

#include <l3xz_io/IoNode.h>

#include <iomanip>

#include <l3xz_io/driver/ssc32/SSC32.h>

#include <l3xz_io/const/LegList.h>

#include <l3xz_io/glue/DynamixelIdList.h>

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

static float constexpr INITIAL_COXA_ANGLE_DEG  = 0.0f;
static float constexpr INITIAL_FEMUR_ANGLE_DEG = 0.0f;
static float constexpr INITIAL_TIBIA_ANGLE_DEG = 0.0f;

static float constexpr INITIAL_PAN_ANGLE_DEG   = 0.0f;
static float constexpr INITIAL_TILT_ANGLE_DEG  = 0.0f;

static std::list<LegJointKey> const HYDRAULIC_LEG_JOINT_LIST =
{
  make_key(Leg::LeftFront,   Joint::Femur),
  make_key(Leg::LeftFront,   Joint::Tibia),
  make_key(Leg::LeftMiddle,  Joint::Femur),
  make_key(Leg::LeftMiddle,  Joint::Tibia),
  make_key(Leg::LeftBack,    Joint::Femur),
  make_key(Leg::LeftBack,    Joint::Tibia),
  make_key(Leg::RightBack,   Joint::Femur),
  make_key(Leg::RightBack,   Joint::Tibia),
  make_key(Leg::RightMiddle, Joint::Femur),
  make_key(Leg::RightMiddle, Joint::Tibia),
  make_key(Leg::RightFront,  Joint::Femur),
  make_key(Leg::RightFront,  Joint::Tibia),
};

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

IoNode::IoNode()
: Node("l3xz_io")
, _state{State::Init_SSC32}
, _open_cyphal_can_if("can0", false)
, _open_cyphal_node(_open_cyphal_can_if, get_logger())
, _dynamixel_ctrl{new dynamixel::Dynamixel(DYNAMIXEL_DEVICE_NAME, DYNAMIXEL_PROTOCOL_VERSION, DYNAMIXEL_BAUD_RATE)}
, _mx28_ctrl{new dynamixel::MX28(_dynamixel_ctrl)}
, _hydraulic_pump{_open_cyphal_node, get_logger()}
, _dynamixel_angle_position_writer{}
, _ssc32_valve_writer{std::make_shared<driver::SSC32>(SSC32_DEVICE_NAME, SSC32_BAUDRATE)}
, _leg_ctrl{_open_cyphal_node, get_logger()}
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
  _ssc32_valve_writer.closeAll();
  _ssc32_valve_writer.doBulkWrite();
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void IoNode::timerCallback()
{
  State next_state = _state;
  switch (_state)
  {
  case State::Init_SSC32:         next_state = handle_Init_SSC32(); break;
  case State::Init_Dynamixel:     next_state = handle_Init_Dynamixel(); break;
  case State::Init_LegController: next_state = handle_Init_LegController(); break;
  case State::Active:             next_state = handle_Active(); break;
  }
  _state = next_state;
}

IoNode::State IoNode::handle_Init_SSC32()
{
  _ssc32_valve_writer.closeAll();
  _ssc32_valve_writer.doBulkWrite();

  return State::Init_Dynamixel;
}

IoNode::State IoNode::handle_Init_Dynamixel()
{
  if (!init_dynamixel())
  {
    RCLCPP_ERROR(get_logger(), "failed to initialize all dynamixel servos.");
    rclcpp::shutdown();
  }

  return State::Init_LegController;
}

IoNode::State IoNode::handle_Init_LegController()
{
  bool all_leg_ctrl_active_heartbeat = true,
       all_leg_ctrl_mode_operational = true,
       all_leg_ctrl_health_nominal   = true;

  for (auto leg : LEG_LIST)
  {
    if (_leg_ctrl.isHeartbeatTimeout(leg, std::chrono::seconds(5)))
    {
      all_leg_ctrl_active_heartbeat = false;
      RCLCPP_ERROR(get_logger(), "LEG_CTRL %d: no heartbeat since 5 seconds", static_cast<int>(glue::LegController::toNodeId(leg)));
    }

    if (!_leg_ctrl.isModeOperational(leg))
    {
      all_leg_ctrl_mode_operational = false;
      RCLCPP_ERROR(get_logger(), "LEG_CTRL %d: mode not operational", static_cast<int>(glue::LegController::toNodeId(leg)));
    }

    if (!_leg_ctrl.isHealthNominal(leg))
    {
      all_leg_ctrl_health_nominal = false;
      RCLCPP_ERROR(get_logger(), "LEG_CTRL %d: health not nominal", static_cast<int>(glue::LegController::toNodeId(leg)));
    }
  }

  if (all_leg_ctrl_active_heartbeat && all_leg_ctrl_mode_operational && all_leg_ctrl_health_nominal)
    return State::Active;

  return State::Init_LegController;
}

IoNode::State IoNode::handle_Active()
{
  /**************************************************************************************
   * CHECK HEARTBEAT/MODE/HEALTH of OpenCyphalDevice's
   **************************************************************************************/

  for (auto leg : LEG_LIST)
  {
    if (_leg_ctrl.isHeartbeatTimeout(leg, std::chrono::seconds(5)))
      RCLCPP_ERROR(get_logger(), "LEG_CTRL %d: no heartbeat since 5 seconds", static_cast<int>(glue::LegController::toNodeId(leg)));
   
    if (!_leg_ctrl.isModeOperational(leg))
      RCLCPP_ERROR(get_logger(), "LEG_CTRL %d: mode not operational", static_cast<int>(glue::LegController::toNodeId(leg)));
   
    if (!_leg_ctrl.isHealthNominal(leg))
      RCLCPP_ERROR(get_logger(), "LEG_CTRL %d: health not nominal", static_cast<int>(glue::LegController::toNodeId(leg)));
  }

  /**************************************************************************************
   * READ FROM PERIPHERALS
   **************************************************************************************/

  auto const [dynamixel_leg_joint_angle_position, dynamixel_head_joint_angle_position] = glue::DynamixelAnglePositionReader::doBulkRead(_mx28_ctrl, get_logger());

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

  leg_angle_actual_msg.femur_angle_deg[0] = _leg_ctrl.femurAngle_deg(Leg::LeftFront);
  leg_angle_actual_msg.femur_angle_deg[1] = _leg_ctrl.femurAngle_deg(Leg::LeftMiddle);
  leg_angle_actual_msg.femur_angle_deg[2] = _leg_ctrl.femurAngle_deg(Leg::LeftBack);
  leg_angle_actual_msg.femur_angle_deg[3] = _leg_ctrl.femurAngle_deg(Leg::RightBack);
  leg_angle_actual_msg.femur_angle_deg[4] = _leg_ctrl.femurAngle_deg(Leg::RightMiddle);
  leg_angle_actual_msg.femur_angle_deg[5] = _leg_ctrl.femurAngle_deg(Leg::RightFront);

  leg_angle_actual_msg.tibia_angle_deg[0] = _leg_ctrl.tibiaAngle_deg(Leg::LeftFront);
  leg_angle_actual_msg.tibia_angle_deg[1] = _leg_ctrl.tibiaAngle_deg(Leg::LeftMiddle);
  leg_angle_actual_msg.tibia_angle_deg[2] = _leg_ctrl.tibiaAngle_deg(Leg::LeftBack);
  leg_angle_actual_msg.tibia_angle_deg[3] = _leg_ctrl.tibiaAngle_deg(Leg::RightBack);
  leg_angle_actual_msg.tibia_angle_deg[4] = _leg_ctrl.tibiaAngle_deg(Leg::RightMiddle);
  leg_angle_actual_msg.tibia_angle_deg[5] = _leg_ctrl.tibiaAngle_deg(Leg::RightFront);

  _leg_angle_pub->publish(leg_angle_actual_msg);

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


  bool turn_hydraulic_pump_on = false;
  for (auto [leg, joint] : HYDRAULIC_LEG_JOINT_LIST)
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
   * WRITE TO PERIPHERALS
   **************************************************************************************/

  if (!_dynamixel_angle_position_writer.doBulkWrite(_mx28_ctrl))
    RCLCPP_ERROR(get_logger(), "failed to set target angles for all dynamixel servos");

  _hydraulic_pump.doWrite();
  _ssc32_valve_writer.doBulkWrite();

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
  for (auto req_id : glue::DYNAMIXEL_ID_LIST)
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

  return true;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
