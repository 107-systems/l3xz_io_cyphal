/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ros_cyphal_bridge/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_ros_cyphal_bridge/CanManager.h>

#include <unistd.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

CanManager::CanManager(rclcpp::Logger const logger, std::string const & iface_name, OnCanFrameReceivedFunc on_can_frame_received)
: _logger{logger}
, IFACE_NAME{iface_name}
, IS_CAN_FD{false}
, _socket_can_fd{socketcanOpen(IFACE_NAME.c_str(), IS_CAN_FD)}
, _on_can_frame_received{on_can_frame_received}
, _rx_thread_active{false}
, _rx_thread{[this]() { this->rx_thread_func(); }}
{
  if (_socket_can_fd < 0) {
    RCLCPP_ERROR(_logger, "Error opening CAN interface '%s'.", iface_name.c_str());
    rclcpp::shutdown();
  }
}

CanManager::~CanManager()
{
  _rx_thread_active = false;
  _rx_thread.join();
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void CanManager::rx_thread_func()
{
  _rx_thread_active = true;

  while (_rx_thread_active)
  {
    CanardFrame rx_frame;
    uint8_t payload_buffer[CANARD_MTU_CAN_CLASSIC] = {0};

    int16_t const rc_blocking = socketcanPop(_socket_can_fd, &rx_frame, sizeof(payload_buffer), payload_buffer, CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, nullptr);

    if (rc_blocking < 0)
    {
      RCLCPP_ERROR(_logger, "'socketcanPop' failed with error %s.", strerror(abs(rc_blocking)));

      /* Perform a timed retry until the interface does become
       * available again, adding a layer of resilience before
       * the software fails completely.
       */
      for(size_t retry_cnt = 0; ; retry_cnt++)
      {
        /* Close (if possible) and invalidate the the file descriptor
         * before starting the process of re-establishing connection.
         */
        close(_socket_can_fd);
        _socket_can_fd = socketcanOpen(IFACE_NAME.c_str(), IS_CAN_FD);

        if (_socket_can_fd < 0) {
          RCLCPP_ERROR(_logger,
                       "[Retry #%ld] 'socketcanOpen(\"%s\", %d)' failed with error %s.",
                       retry_cnt,
                       IFACE_NAME.c_str(),
                       IS_CAN_FD,
                       strerror(abs(_socket_can_fd)));

          /* Wait a little bit before the next retry. */
          std::this_thread::sleep_for(std::chrono::seconds(1));

          continue;
        }

        /* Attempt a non-blocking read to see if that fails again, if so
         * print an error and attempt a delayed re-connect, otherwise
         * leave this loop.
         */
        int16_t const rc_non_blocking = socketcanPop(_socket_can_fd, &rx_frame, sizeof(payload_buffer), payload_buffer, 0, nullptr);
        if (rc_non_blocking < 0)
        {
          RCLCPP_ERROR(_logger,
                       "[Retry #%ld] 'socketcanPop' failed with error %s.",
                       retry_cnt,
                       strerror(abs(rc_blocking)));

          /* Wait a little bit before the next retry. */
          std::this_thread::sleep_for(std::chrono::seconds(1));

          continue;
        }
        else if (rc_non_blocking == 0) /* Timeout. */
          break;
        else if (rc_non_blocking > 0) { /* Frame received. */
          _on_can_frame_received(rx_frame);
          break;
        }
      }

      RCLCPP_INFO(_logger, "Re-opening CAN device succeeded.");
    }
    else if (rc_blocking == 0)
      RCLCPP_DEBUG(_logger, "'socketcanPop' receive time-out (this is expected if no CAN messages are being received).");
    else
      _on_can_frame_received(rx_frame);
  }

  /* Cleanup. */
  close(_socket_can_fd);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
