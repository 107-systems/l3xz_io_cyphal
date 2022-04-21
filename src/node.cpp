/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
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

#include <l3xz/Const.h>

#include <l3xz/driver/dynamixel/MX28.h>
#include <l3xz/driver/dynamixel/Dynamixel.h>

#include <l3xz/common/interface/sensor/AnglePositionSensor.h>

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

bool init_dynamixel  (driver::SharedMX28 & mx28_ctrl);
void deinit_dynamixel(driver::SharedMX28 & mx28_ctrl);

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr & msg, l3xz::TeleopCommandData & teleop_cmd_data);

/**************************************************************************************
 * CONSTANT
 **************************************************************************************/

static std::string const DYNAMIXEL_DEVICE_NAME = "/dev/ttyUSB0";
static float       const DYNAMIXEL_PROTOCOL_VERSION = 2.0f;
static int         const DYNAMIXEL_BAUD_RATE = 115200;

static driver::Dynamixel::IdVect const DYNAMIXEL_ID_VECT{1,2,3,4,5,6,7,8};

static driver::MX28::AngleDataSet const L3XZ_INITIAL_ANGLE_DATA_SET =
{
  {1, 180.0f},
  {2, 180.0f},
  {3, 180.0f},
  {4, 180.0f},
  {5, 180.0f},
  {6, 180.0f},
  {7, 180.0f},
  {8, 180.0f},
};

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv) try
{
  ros::init(argc, argv, "l3xz");

  ros::NodeHandle node_hdl;


  std::shared_ptr<driver::Dynamixel> dynamixel_ctrl = std::make_shared<driver::Dynamixel>(DYNAMIXEL_DEVICE_NAME, DYNAMIXEL_PROTOCOL_VERSION, DYNAMIXEL_BAUD_RATE);
  driver::SharedMX28 mx28_ctrl = std::make_shared<driver::MX28>(dynamixel_ctrl);

  if (!init_dynamixel(mx28_ctrl))
    ROS_ERROR("init_dynamixel failed.");
  ROS_INFO("init_dynamixel successfully completed.");


  l3xz::TeleopCommandData teleop_cmd_data;
  ros::Subscriber cmd_vel_sub = node_hdl.subscribe<geometry_msgs::Twist>("/l3xz/cmd_vel", 10, std::bind(cmd_vel_callback, std::placeholders::_1, std::ref(teleop_cmd_data)));


  auto coxa_leg_front_left   = std::make_shared<common::interface::sensor::AnglePositionSensor>("LEG F/L Coxa");
  auto coxa_leg_front_right  = std::make_shared<common::interface::sensor::AnglePositionSensor>("LEG F/R Coxa");
  auto coxa_leg_middle_left  = std::make_shared<common::interface::sensor::AnglePositionSensor>("LEG M/L Coxa");
  auto coxa_leg_middle_right = std::make_shared<common::interface::sensor::AnglePositionSensor>("LEG M/R Coxa");
  auto coxa_leg_back_left    = std::make_shared<common::interface::sensor::AnglePositionSensor>("LEG B/L Coxa");
  auto coxa_leg_back_right   = std::make_shared<common::interface::sensor::AnglePositionSensor>("LEG B/R Coxa");
  auto sensor_head_pan       = std::make_shared<common::interface::sensor::AnglePositionSensor>("HEAD Pan    ");
  auto sensor_head_tilt      = std::make_shared<common::interface::sensor::AnglePositionSensor>("HEAD Tilt   ");

  static std::map<driver::Dynamixel::Id, common::interface::sensor::SharedAnglePositionSensor> const DYNAMIXEL_ID_TO_ANGLE_POSITION_SENSOR =
  {
    {1, coxa_leg_front_left},
    {2, coxa_leg_front_right},
    {3, coxa_leg_middle_left},
    {4, coxa_leg_middle_right},
    {5, coxa_leg_back_left},
    {6, coxa_leg_back_right},
    {7, sensor_head_pan},
    {8, sensor_head_tilt},
  };

  driver::MX28::AngleDataSet l3xz_mx28_target_angle = L3XZ_INITIAL_ANGLE_DATA_SET;

  for (ros::Rate loop_rate(50);
       ros::ok();
       loop_rate.sleep())
  {
    /* Simultaneously read the current angle from all dynamixel servos and update the angle position sensors. */
    for (auto [id, angle_deg] : mx28_ctrl->getAngle(DYNAMIXEL_ID_VECT))
      DYNAMIXEL_ID_TO_ANGLE_POSITION_SENSOR.at(id)->update(angle_deg);

    ROS_DEBUG("L3XZ Dynamixel Current Angles:\n  %s\n  %s\n  %s\n  %s\n  %s\n  %s\n  %s\n  %s",
      coxa_leg_front_left->toStr().c_str(),
      coxa_leg_front_right->toStr().c_str(),
      coxa_leg_middle_left->toStr().c_str(),
      coxa_leg_middle_right->toStr().c_str(),
      coxa_leg_back_left->toStr().c_str(),
      coxa_leg_back_right->toStr().c_str(),
      sensor_head_pan->toStr().c_str(),
      sensor_head_tilt->toStr().c_str());

    /* Calculate new values for sensor head, both pan and tilt joint
     * based on the input provided by the teleop node.
     */
    static float const MAX_ANGLE_INCREMENT_PER_CYCLE_DEG = 10.0f;

    float const sensor_head_pan_actual = sensor_head_pan->get().value();
    float const sensor_head_pan_target = sensor_head_pan_actual + (teleop_cmd_data.angular_velocity_head_pan * MAX_ANGLE_INCREMENT_PER_CYCLE_DEG);
    l3xz_mx28_target_angle.at(7) = sensor_head_pan_target;

    float const sensor_head_tilt_actual = sensor_head_tilt->get().value();
    float const sensor_head_tilt_target = sensor_head_tilt_actual + (teleop_cmd_data.angular_velocity_head_tilt * MAX_ANGLE_INCREMENT_PER_CYCLE_DEG);
    l3xz_mx28_target_angle.at(8) = sensor_head_tilt_target;

    ROS_INFO("Head\n  Pan : actual = %.2f, target = %.2f\n  Tilt: actual = %.2f, target = %.2f", sensor_head_pan_actual, sensor_head_pan_target, sensor_head_tilt_actual, sensor_head_tilt_target);



    if (!mx28_ctrl->setAngle(l3xz_mx28_target_angle))
      ROS_ERROR("failed to set target angles for all dynamixel servos");

    ros::spinOnce();
/*
    mx28_ctrl->turnLedOn(opt_id_vect.value());
    loop_rate.sleep();
    mx28_ctrl->turnLedOff(opt_id_vect.value());
    loop_rate.sleep();

    driver::MX28::AngleDataVect angle_vect_act = mx28_ctrl->getAngle(opt_id_vect.value());
    for (auto [id, angle_deg] : angle_vect_act)
      ROS_INFO("[ID:%03d] Angle Act = %0.02f deg", id, angle_deg);

    driver::MX28::AngleDataVect angle_vect_set = angle_vect_act;
    std::transform(angle_vect_act.begin(),
                   angle_vect_act.end(),
                   angle_vect_set.begin(),
                   [](std::tuple<uint8_t, float> const & in) -> std::tuple<uint8_t, float>
                   {
                     auto [id, angle_set] = in;
                     angle_set += 10.0f;
                     while (angle_set > 360.0f)
                      angle_set -= 360.0f;
                     return std::make_tuple(id, angle_set);
                   });

    for (auto [id, angle_deg] : angle_vect_set)
      ROS_INFO("[ID:%03d] Angle Set = %0.02f deg", id, angle_deg);


    if (!mx28_ctrl->setAngle(angle_vect_set))
      ROS_ERROR("setAngle() failed");
*/
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
  for (auto req_id : DYNAMIXEL_ID_VECT)
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

  auto isInitialTargetAngleReached = [](driver::MX28::AngleDataSet const & current_angle)
  {
    for (auto [actual_id, actual_angle_deg] : current_angle)
    {
      if (!L3XZ_INITIAL_ANGLE_DATA_SET.count(actual_id)) {
        ROS_ERROR("Could not find ID for angle comparison");
        return false;
      }

      float const EPSILON = 1.0f;
      float const set_angle_deg = L3XZ_INITIAL_ANGLE_DATA_SET.at(actual_id);
      float const abs_angle_diff = fabs(actual_angle_deg - set_angle_deg);

      if (abs_angle_diff > EPSILON) {
        ROS_INFO("Not yet target angle reached for ID %d (set: %.2f, actual: %.2f)", actual_id, set_angle_deg, actual_angle_deg);
        return false;
      }
    }
    return true;
  };

  mx28_ctrl->torqueOn(DYNAMIXEL_ID_VECT);

  if (!mx28_ctrl->setAngle(L3XZ_INITIAL_ANGLE_DATA_SET)) {
    ROS_ERROR("failed to set initial angles for all dynamixel servos");
    return false;
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));

  if (!isInitialTargetAngleReached(mx28_ctrl->getAngle(DYNAMIXEL_ID_VECT))) {
    ROS_ERROR("failed to set all dynamixel servos to initial position");
    return false;
  }

  return true;
}

void deinit_dynamixel(driver::SharedMX28 & mx28_ctrl)
{
  mx28_ctrl->torqueOff(DYNAMIXEL_ID_VECT);
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr & msg, l3xz::TeleopCommandData & teleop_cmd_data)
{
  teleop_cmd_data.linear_velocity_x           = msg->linear.x;
  teleop_cmd_data.linear_velocity_y           = msg->linear.y;
  teleop_cmd_data.angular_velocity_head_tilt  = msg->angular.x;
  teleop_cmd_data.angular_velocity_head_pan   = msg->angular.y;
  teleop_cmd_data.angular_velocity_z          = msg->angular.z;

  ROS_DEBUG("v_tilt = %.2f, v_pan = %.2f", teleop_cmd_data.angular_velocity_head_tilt, teleop_cmd_data.angular_velocity_head_pan);
}
