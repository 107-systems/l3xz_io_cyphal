/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <string>
#include <thread>

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/LaserScan.h>

#include <sweep/sweep.hpp>

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sweep_node");

  ros::NodeHandle node_hdl;
  ros::NodeHandle node_hdl_private("~");

  std::string serial_port, frame_id;
  int rotation_speed, sample_rate;

  node_hdl_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
  node_hdl_private.param<int>("rotation_speed", rotation_speed, 1);
  node_hdl_private.param<int>("sample_rate", sample_rate, 500);
  node_hdl_private.param<std::string>("frame_id", frame_id, "laser_frame");

  ROS_INFO("node config:\n  port : %s\n  speed: %d Hz\n  rate : %d Hz\n  frame: %s", serial_port.c_str(), rotation_speed, sample_rate, frame_id.c_str());

  ros::Publisher scan_pub = node_hdl.advertise<sensor_msgs::LaserScan>(frame_id, 10);

  try
  {
    sweep::sweep scanner(serial_port.c_str());

    ROS_INFO("configuring scanse sweep.");

    scanner.set_motor_speed(rotation_speed);
    while (!scanner.get_motor_ready())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    scanner.set_sample_rate(sample_rate);
    scanner.start_scanning();

    ROS_INFO("starting data aquisition.");

    for (uint32_t sequence_id = 0; ros::ok(); sequence_id++)
    {
      /* Obtain a full scan. */
      sweep::scan const scan = scanner.get_scan();

      sensor_msgs::LaserScan laser_scan_msg;

      /* Populate sensor_msgs/LaserScan header. */
      laser_scan_msg.header.seq = sequence_id;
      laser_scan_msg.header.stamp = ros::Time::now();
      laser_scan_msg.header.frame_id = frame_id;

      /* Populate sensor_msgs/LaserScan data. */
      float const samples_per_rotation = static_cast<float>(sample_rate) / static_cast<float>(rotation_speed);

      laser_scan_msg.angle_min       = 0.0;
      laser_scan_msg.angle_max       = 2.0 * M_PI;
      laser_scan_msg.angle_increment = laser_scan_msg.angle_max / samples_per_rotation;
      laser_scan_msg.time_increment  = 1.0 / static_cast<float>(sample_rate);
      laser_scan_msg.range_min       = 0.0;
      laser_scan_msg.range_max       = 40.0;

      laser_scan_msg.ranges.assign(scan.samples.size(), std::numeric_limits<float>::infinity());

      size_t idx = 0;
      for (auto [angle_milli_deg, distance_mm, signal_strength] : scan.samples) {
        laser_scan_msg.ranges[idx] = static_cast<float>(distance_mm) / 1000.0;
        idx++;
      }

      /* Publish the laser scan. */
      scan_pub.publish(laser_scan_msg);

      ros::spinOnce();
    }

    scanner.stop_scanning();

    return EXIT_SUCCESS;  
  }
  catch(sweep::device_error const & e)
  {
    ROS_ERROR("%s", e.what());
    return EXIT_FAILURE;
  }
}
