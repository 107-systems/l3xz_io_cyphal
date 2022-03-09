/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <dynamixel_sdk.h>

/**************************************************************************************
 * CONSTANT
 **************************************************************************************/

static std::string const DYNAMIXEL_DEVICE_NAME = "/dev/ttyUSB0";
static float       const DYNAMIXEL_PROTOCOL_VERSION = 2.0f;
static int         const DYNAMIXEL_BAUD_RATE = 115200;

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "l3xz");


  dynamixel::PortHandler * portHandler = dynamixel::PortHandler::getPortHandler(DYNAMIXEL_DEVICE_NAME.c_str());
  dynamixel::PacketHandler * packetHandler = dynamixel::PacketHandler::getPacketHandler(DYNAMIXEL_PROTOCOL_VERSION);

  if (!portHandler->openPort())
    ROS_FATAL("libdynamixel:openPort failed.");

  if (!portHandler->setBaudRate(DYNAMIXEL_BAUD_RATE))
    ROS_FATAL("libdynamixel:setBaudRate failed.");

  std::vector<uint8_t> servo_id_vect;
  if (int const res = packetHandler->broadcastPing(portHandler, servo_id_vect); res != COMM_SUCCESS)
    ROS_ERROR("%s", packetHandler->getTxRxResult(res));

  ROS_INFO("Detected Dynamixel:");
  for (uint8_t id : servo_id_vect)
    ROS_INFO("[ID:%03d]", id);

  portHandler->closePort();


  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
