/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <cstring>

#include <iostream>

#include <phy/opencyphal/Node.hpp>
#include <phy/opencyphal/Types.h>
#include <phy/opencyphal/SocketCAN.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace phy::opencyphal;

/**************************************************************************************
 * CONSTANT
 **************************************************************************************/

static uint8_t const THIS_NODE_ID = 0;

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv) try
{
  SocketCAN can_if("can0", false);
  Node node(THIS_NODE_ID,
            [&can_if](CanardFrame const & tx_frame) -> bool
            {
              if (int16_t const rc = can_if.push(&tx_frame, CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC); rc <= 0) {
                std::cerr << "socketcanPush failed with error " << rc << std::endl;
                return false;
              }
              return true;
            });

  node.subscribe<uavcan::node::Heartbeat_1_0<>>([](CanardTransfer const & transfer)
  {
    uavcan::node::Heartbeat_1_0<> const hb = uavcan::node::Heartbeat_1_0<>::deserialize(transfer);
    std::cout << "Heartbeat received"
              << "\n\tMode = "
              << hb.data.mode.value
              << std::endl;
  });

  node.subscribe<uavcan::primitive::scalar::Real32_1_0<1001>>([](CanardTransfer const & transfer)
  {
    uavcan::primitive::scalar::Real32_1_0<1001> const input_voltage = uavcan::primitive::scalar::Real32_1_0<1001>::deserialize(transfer);
    std::cout << "Battery Voltage = "
              << input_voltage.data.value
              << std::endl;
  });

  node.subscribe<uavcan::primitive::scalar::Real32_1_0<1002>>([](CanardTransfer const & transfer)
  {
    uavcan::primitive::scalar::Real32_1_0<1002> const as5048_a_angle = uavcan::primitive::scalar::Real32_1_0<1002>::deserialize(transfer);
    std::cout << "Angle[AS5048 A] = "
              << as5048_a_angle.data.value
              << std::endl;
  });

  node.subscribe<uavcan::primitive::scalar::Real32_1_0<1003>>([](CanardTransfer const & transfer)
  {
    uavcan::primitive::scalar::Real32_1_0<1003> const as5048_b_angle = uavcan::primitive::scalar::Real32_1_0<1003>::deserialize(transfer);
    std::cout << "Angle[AS5048 B] = "
              << as5048_b_angle.data.value
              << std::endl;
  });

  for (;;)
  {
    /* Receive and process received frames. 
     */
    CanardFrame rx_frame;
    uint8_t payload_buffer[64] = {0};
    
    int16_t const rc = can_if.pop(&rx_frame, sizeof(payload_buffer), payload_buffer, CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, nullptr);
    if (rc < 0) {
      std::cerr << "socketcanPop failed with error " << strerror(abs(rc)) << std::endl;
    }
    else if (rc == 0) {
      std::cerr << "socketcanPop timeout while receiving." << std::endl;
    }
    else {
      node.onCanFrameReceived(rx_frame);
    }
  }


  return EXIT_SUCCESS;
}
catch(std::exception const & e)
{
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}

/**************************************************************************************
 * FUNCTION IMPLEMENTATION
 **************************************************************************************/
