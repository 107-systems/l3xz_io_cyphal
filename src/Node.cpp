/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/ros2_cyphal_bridge/graphs/contributors.
 */

/**************************************************************************************
 * INCcdLUDES
 **************************************************************************************/

#include <ros2_cyphal_bridge/Node.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Node::Node()
: rclcpp::Node("ros2_cyphal_bridge")
, _node_heap{}
, _node_hdl{_node_heap.data(),
            _node_heap.size(),
            [this] () { return micros(); },
            [this] (CanardFrame const & frame) { return _can_mgr->transmit(frame); },
            ::Node::DEFAULT_NODE_ID,
            CYPHAL_TX_QUEUE_SIZE,
            CYPHAL_RX_QUEUE_SIZE,
            ::Node::DEFAULT_MTU_SIZE}
, _node_mtx{}
, _node_start{std::chrono::steady_clock::now()}
, _prev_heartbeat_timepoint{std::chrono::steady_clock::now()}
, _prev_io_loop_timepoint{std::chrono::steady_clock::now()}
{
  init_heartbeat();
  init_cyphal_heartbeat();
  init_cyphal_node_info();

  init_cyphal_to_ros_angle_actual();
  init_cyphal_to_ros_tibia_endpoint_switch();
  init_cyphal_to_ros_estop();
  init_cyphal_to_ros_radiation_tick_cnt();
  init_ros_to_cyphal_light_mode();
  init_ros_to_cyphal_servo_pulse_width();
  init_ros_to_cyphal_pump_readiness();
  init_ros_to_cyphal_pump_setpoint();

  declare_parameter("can_iface", "can0");
  declare_parameter("can_node_id", 100);

  RCLCPP_INFO(get_logger(),
              "configuring CAN bus:\n\tDevice: %s\n\tNode Id: %ld",
              get_parameter("can_iface").as_string().c_str(),
              get_parameter("can_node_id").as_int());

  _node_hdl.setNodeId(get_parameter("can_node_id").as_int());

  _can_mgr = std::make_unique<CanManager>(
    get_logger(),
    get_parameter("can_iface").as_string(),
    [this](CanardFrame const & frame)
    {
      std::lock_guard<std::mutex> lock(_node_mtx);
      _node_hdl.onCanFrameReceived(frame);
    });

  /* Configure periodic control loop function. */
  _io_loop_timer = create_wall_timer(IO_LOOP_RATE, [this]() { this->io_loop(); });

  RCLCPP_INFO(get_logger(), "%s init complete.", get_name());
}

Node::~Node()
{
  RCLCPP_INFO(get_logger(), "%s shut down successfully.", get_name());
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Node::init_heartbeat()
{
  std::stringstream heartbeat_topic;
  heartbeat_topic << "/l3xz/" << get_name() << "/heartbeat";

  _heartbeat_pub = heartbeat::Publisher::create(*this, heartbeat_topic.str(), HEARTBEAT_LOOP_RATE);
}

void Node::init_cyphal_heartbeat()
{
  _cyphal_heartbeat_pub = _node_hdl.create_publisher<uavcan::node::Heartbeat_1_0>(1*1000*1000UL /* = 1 sec in usecs. */);
}

void Node::init_cyphal_node_info()
{
  _cyphal_node_info = _node_hdl.create_node_info(
    /* uavcan.node.Version.1.0 protocol_version */
    1, 0,
    /* uavcan.node.Version.1.0 hardware_version */
    1, 0,
    /* uavcan.node.Version.1.0 software_version */
    0, 1,
    /* saturated uint64 software_vcs_revision_id */
#ifdef CYPHAL_NODE_INFO_GIT_VERSION
    CYPHAL_NODE_INFO_GIT_VERSION,
#else
    0,
#endif
    /* saturated uint8[16] unique_id */
    std::array<uint8_t, 16>{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F},
    /* saturated uint8[<=50] name */
    "107-systems.ros2_cyphal_bridge"
  );
}

void Node::io_loop()
{
  auto const now = std::chrono::steady_clock::now();
  auto const io_loop_rate = (now - _prev_io_loop_timepoint);
  if (io_loop_rate > (IO_LOOP_RATE + std::chrono::milliseconds(1)))
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         10 * 1000UL, /* 10 sec. */
                         "io_loop should be called every %ld ms, but is %ld ms instead",
                         IO_LOOP_RATE.count(),
                         std::chrono::duration_cast<std::chrono::milliseconds>(io_loop_rate).count());
  _prev_io_loop_timepoint = now;

  std::lock_guard<std::mutex> lock(_node_mtx);

  _node_hdl.spinSome();

  if ((now - _prev_heartbeat_timepoint) > CYPHAL_HEARTBEAT_PERIOD)
  {
    uavcan::node::Heartbeat_1_0 msg;

    msg.uptime = std::chrono::duration_cast<std::chrono::seconds>(now - _node_start).count();
    msg.health.value = uavcan::node::Health_1_0::NOMINAL;
    msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
    msg.vendor_specific_status_code = 0;

    _cyphal_heartbeat_pub->publish(msg);

    _prev_heartbeat_timepoint = now;
  }
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::init_cyphal_to_ros_angle_actual()
{
  std::map<CanardPortID, std::string> const ANGLE_ACTUAL_PORT_ID_to_TOPIC =
    {
      {1001U, "/l3xz/leg/left_front/femur/angle/actual"},
      {1002U, "/l3xz/leg/left_front/tibia/angle/actual"},
      {1004U, "/l3xz/leg/left_middle/femur/angle/actual"},
      {1005U, "/l3xz/leg/left_middle/tibia/angle/actual"},
      {1007U, "/l3xz/leg/left_back/femur/angle/actual"},
      {1008U, "/l3xz/leg/left_back/tibia/angle/actual"},
      {1010U, "/l3xz/leg/right_back/femur/angle/actual"},
      {1011U, "/l3xz/leg/right_back/tibia/angle/actual"},
      {1013U, "/l3xz/leg/right_middle/femur/angle/actual"},
      {1014U, "/l3xz/leg/right_middle/tibia/angle/actual"},
      {1016U, "/l3xz/leg/right_front/femur/angle/actual"},
      {1017U, "/l3xz/leg/right_front/tibia/angle/actual"},
    };

  for (auto [port_id, ros_topic] : ANGLE_ACTUAL_PORT_ID_to_TOPIC)
  {
    _angle_actual_ros_pub[port_id] = create_publisher<std_msgs::msg::Float32>(ros_topic, 1);

    _angle_actual_cyphal_sub[port_id] = _node_hdl.create_subscription<uavcan::si::unit::angle::Scalar_1_0>(
      port_id,
      [this, port_id](uavcan::si::unit::angle::Scalar_1_0 const & msg)
      {
        std_msgs::msg::Float32 angle_actual_rad_msg;
        angle_actual_rad_msg.data = msg.radian;
        _angle_actual_ros_pub.at(port_id)->publish(angle_actual_rad_msg);
      });

    RCLCPP_INFO(get_logger(), "Mapping [%d] to \"%s\"", port_id, ros_topic.c_str());
  }
}

void Node::init_cyphal_to_ros_tibia_endpoint_switch()
{
  std::map<CanardPortID, std::string> const TIBIA_ENDPOINT_SWITCH_ACTUAL_PORT_ID_to_TOPIC =
    {
      {1003U, "/l3xz/leg/left_front/tibia_endpoint_switch/actual"},
      {1006U, "/l3xz/leg/left_middle/tibia_endpoint_switch/actual"},
      {1009U, "/l3xz/leg/left_back/tibia_endpoint_switch/actual"},
      {1012U, "/l3xz/leg/right_back/tibia_endpoint_switch/actual"},
      {1015U, "/l3xz/leg/right_middle/tibia_endpoint_switch/actual"},
      {1018U, "/l3xz/leg/right_front/tibia_endpoint_switch/actual"},
    };

  for (auto [port_id, ros_topic] : TIBIA_ENDPOINT_SWITCH_ACTUAL_PORT_ID_to_TOPIC)
  {
    _tibia_endpoint_switch_ros_pub[port_id] = create_publisher<std_msgs::msg::Bool>(ros_topic, 1);

    _tibia_endpoint_switch_cyphal_sub[port_id] = _node_hdl.create_subscription<uavcan::primitive::scalar::Bit_1_0>(
      port_id,
      [this, port_id](uavcan::primitive::scalar::Bit_1_0 const & msg)
      {
        std_msgs::msg::Bool tibia_endpoint_switch_msg;
        tibia_endpoint_switch_msg.data = msg.value;
        _tibia_endpoint_switch_ros_pub.at(port_id)->publish(tibia_endpoint_switch_msg);
      });

    RCLCPP_INFO(get_logger(), "Mapping [%d] to \"%s\"", port_id, ros_topic.c_str());
  }
}

void Node::init_cyphal_to_ros_estop()
{
  std::string const ROS_TOPIC = "/l3xz/estop/actual";
  CanardPortID const PORT_ID = 2001U;

  _estop_ros_pub = create_publisher<std_msgs::msg::Bool>(ROS_TOPIC, 1);

  _estop_cyphal_sub = _node_hdl.create_subscription<uavcan::primitive::scalar::Bit_1_0>(
    PORT_ID,
    [this](uavcan::primitive::scalar::Bit_1_0 const & msg)
    {
      std_msgs::msg::Bool estop_msg;
      estop_msg.data = msg.value;
      _estop_ros_pub->publish(estop_msg);
    });

  RCLCPP_INFO(get_logger(), "Mapping [%d] to \"%s\"", PORT_ID, ROS_TOPIC.c_str());
}

void Node::init_cyphal_to_ros_radiation_tick_cnt()
{
  std::string const ROS_TOPIC = "/l3xz/radiation/actual";
  CanardPortID const PORT_ID = 3001U;

  _radiation_tick_cnt_ros_pub = create_publisher<std_msgs::msg::Int16>(ROS_TOPIC, 1);

  _radiation_tick_cnt_cyphal_sub = _node_hdl.create_subscription<uavcan::primitive::scalar::Natural16_1_0>(
    PORT_ID,
    [this](uavcan::primitive::scalar::Natural16_1_0 const & msg)
    {
      std_msgs::msg::Int16 radiation_tick_msg;
      radiation_tick_msg.data = msg.value;
      _radiation_tick_cnt_ros_pub->publish(radiation_tick_msg);
    });

  RCLCPP_INFO(get_logger(), "Mapping [%d] to \"%s\"", PORT_ID, ROS_TOPIC.c_str());
}

void Node::init_ros_to_cyphal_light_mode()
{
  std::string const ROS_TOPIC = "/l3xz/light_mode/target";
  CanardPortID const PORT_ID = 2002U;

  _light_mode_cyphal_pub = _node_hdl.create_publisher<uavcan::primitive::scalar::Integer8_1_0>(PORT_ID, 1*1000*1000UL /* = 1 sec in usecs. */);

  _light_mode_ros_sub = create_subscription<std_msgs::msg::Int8>(
    ROS_TOPIC,
    1,
    [this](std_msgs::msg::Int8::SharedPtr const msg)
    {
      uavcan::primitive::scalar::Integer8_1_0 light_mode_msg;
      light_mode_msg.value = msg->data;
      _light_mode_cyphal_pub->publish(light_mode_msg);
    });

  RCLCPP_INFO(get_logger(), "Mapping [%d] to \"%s\"", PORT_ID, ROS_TOPIC.c_str());
}

void Node::init_ros_to_cyphal_servo_pulse_width()
{
  std::string const ROS_TOPIC = "/l3xz/servo_pulse_width/target";
  CanardPortID const PORT_ID = 4001U;

  _servo_pulse_width_cyphal_pub = _node_hdl.create_publisher<uavcan::primitive::array::Natural16_1_0>(PORT_ID, 1*1000*1000UL /* = 1 sec in usecs. */);

  _servo_pulse_width_ros_sub = create_subscription<std_msgs::msg::UInt16MultiArray>(
    ROS_TOPIC,
    1,
    [this](std_msgs::msg::UInt16MultiArray::SharedPtr const msg)
    {
      uavcan::primitive::array::Natural16_1_0 pulse_width_msg;

      for (size_t i = 0; i < msg->layout.dim[0].size; i++)
      {
        uint16_t const pulse_width_us = msg->data[i];
        pulse_width_msg.value.push_back(pulse_width_us);
      }

      _servo_pulse_width_cyphal_pub->publish(pulse_width_msg);
    });

  RCLCPP_INFO(get_logger(), "Mapping [%d] to \"%s\"", PORT_ID, ROS_TOPIC.c_str());
}

void Node::init_ros_to_cyphal_pump_readiness()
{
  std::string const ROS_TOPIC = "/l3xz/pump/readiness/target";
  CanardPortID const PORT_ID = 5001U;

  _pump_readiness_cyphal_pub = _node_hdl.create_publisher<reg::udral::service::common::Readiness_0_1>(PORT_ID, 1*1000*1000UL /* = 1 sec in usecs. */);

  _pump_readiness_ros_sub = create_subscription<std_msgs::msg::Int8>(
    ROS_TOPIC,
    1,
    [this](std_msgs::msg::Int8::SharedPtr const msg)
    {
      reg::udral::service::common::Readiness_0_1 readiness_msg;
      readiness_msg.value = msg->data;
      _pump_readiness_cyphal_pub->publish(readiness_msg);
    });

  RCLCPP_INFO(get_logger(), "Mapping [%d] to \"%s\"", PORT_ID, ROS_TOPIC.c_str());
}

void Node::init_ros_to_cyphal_pump_setpoint()
{
  std::string const ROS_TOPIC = "/l3xz/pump/rpm/target";
  CanardPortID const PORT_ID = 5002U;

  _pump_rpm_setpoint_cyphal_pub = _node_hdl.create_publisher<reg::udral::service::actuator::common::sp::Scalar_0_1>(PORT_ID, 1*1000*1000UL /* = 1 sec in usecs. */);

  _pump_rpm_setpoint_ros_sub = create_subscription<std_msgs::msg::Float32>(
    ROS_TOPIC,
    1,
    [this](std_msgs::msg::Float32::SharedPtr const msg)
    {
      reg::udral::service::actuator::common::sp::Scalar_0_1 rpm_setpoint_msg;
      rpm_setpoint_msg.value = msg->data;
      _pump_rpm_setpoint_cyphal_pub->publish(rpm_setpoint_msg);
    });

  RCLCPP_INFO(get_logger(), "Mapping [%d] to \"%s\"", PORT_ID, ROS_TOPIC.c_str());
}

CanardMicrosecond Node::micros()
{
  auto const now = std::chrono::steady_clock::now();
  auto const node_uptime = (now - _node_start);
  return std::chrono::duration_cast<std::chrono::microseconds>(node_uptime).count();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
