/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/ros2_cyphal_bridge/graphs/contributors.
 */

/**************************************************************************************
 * INCcdLUDES
 **************************************************************************************/

#include <ros2_cyphal_bridge/Node.h>

#include <ros2_cyphal_bridge/const/LegList.h>
#include <ros2_cyphal_bridge/const/JointList.h>
#include <ros2_cyphal_bridge/types/LegJoint.h>

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
, _prev_io_loop_timepoint{std::chrono::steady_clock::now()}
{
  init_cyphal_to_ros();

  declare_parameter("can_iface", "can0");
  declare_parameter("can_node_id", 100);

  RCLCPP_INFO(get_logger(),
              "configuring CAN bus:\n\tDevice: %s\n\tNode Id: %ld",
              get_parameter("can_iface").as_string().c_str(),
              get_parameter("can_node_id").as_int());

  _node_hdl.setNodeId(get_parameter("can_node_id").as_int());

  _can_mgr = std::unique_ptr<CanManager>(new CanManager(
    get_logger(),
    get_parameter("can_iface").as_string(),
    [this](CanardFrame const & frame)
    {
      std::lock_guard<std::mutex> lock(_node_mtx);
      _node_hdl.onCanFrameReceived(frame);
    }));

  /* Configure periodic control loop function. */
  _io_loop_timer = create_wall_timer
    (std::chrono::milliseconds(IO_LOOP_RATE.count()),
     [this]() { this->io_loop(); });

  RCLCPP_INFO(get_logger(), "Node started successfully.");
}

Node::~Node()
{
  RCLCPP_INFO(get_logger(), "Node shut down successfully.");
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

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

  {
    std::lock_guard<std::mutex> lock(_node_mtx);
    _node_hdl.spinSome();
  }
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::init_cyphal_to_ros()
{
  RCLCPP_INFO(get_logger(), "Mapping Cyphal/CAN femur/tibia angle actual to ROS topics:");
  for (auto leg : LEG_LIST)
    for (auto joint : JOINT_LIST)
    {
      static std::map<LegJointKey, CanardPortID> const ANGLE_ACTUAL_PORT_ID_MAP =
        {
          {make_key(Leg::LeftFront,   Joint::Femur), 1001U},
          {make_key(Leg::LeftFront,   Joint::Tibia), 1002U},
          {make_key(Leg::LeftMiddle,  Joint::Femur), 1004U},
          {make_key(Leg::LeftMiddle,  Joint::Tibia), 1005U},
          {make_key(Leg::LeftBack,    Joint::Femur), 1007U},
          {make_key(Leg::LeftBack,    Joint::Tibia), 1008U},
          {make_key(Leg::RightBack,   Joint::Femur), 1010U},
          {make_key(Leg::RightBack,   Joint::Tibia), 1011U},
          {make_key(Leg::RightMiddle, Joint::Femur), 1013U},
          {make_key(Leg::RightMiddle, Joint::Tibia), 1014U},
          {make_key(Leg::RightFront,  Joint::Femur), 1016U},
          {make_key(Leg::RightFront,  Joint::Tibia), 1017U},
        };

      std::stringstream angle_actual_pub_topic;
      angle_actual_pub_topic << "/l3xz/leg/" << LegToStr(leg) << "/" << JointToStr(joint) << "/angle/actual";

      CanardPortID const port_id = ANGLE_ACTUAL_PORT_ID_MAP.at(make_key(leg, joint));

      _angle_actual_ros_pub[port_id] = create_publisher<std_msgs::msg::Float32>(angle_actual_pub_topic.str(), 1);

      _angle_actual_cyphal_sub[port_id] = _node_hdl.create_subscription<uavcan::si::unit::angle::Scalar_1_0>(
        port_id,
        [this, port_id](uavcan::si::unit::angle::Scalar_1_0 const & msg)
        {
          std_msgs::msg::Float32 angle_actual_rad_msg;
          angle_actual_rad_msg.data = msg.radian;
          _angle_actual_ros_pub.at(port_id)->publish(angle_actual_rad_msg);
        });

      RCLCPP_INFO(get_logger(), "Mapping [%d] to \"%s\"", port_id, angle_actual_pub_topic.str().c_str());
    }
}

CanardMicrosecond Node::micros()
{
  ::timespec ts{};
  if (0 != clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts))
  {
    RCLCPP_ERROR(get_logger(), "CLOCK_PROCESS_CPUTIME_ID");
    rclcpp::shutdown();
  }
  auto const nsec = (ts.tv_sec * 1000*1000*1000UL) + ts.tv_nsec;
  return static_cast<CanardMicrosecond>(nsec / 1000UL);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
