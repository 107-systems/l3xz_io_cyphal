/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ros_cyphal_bridge/graphs/contributors.
 */

#ifndef PHY_SERIAL_ASYNCSERIAL_H_
#define PHY_SERIAL_ASYNCSERIAL_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <cstdint>

#include <mutex>
#include <future>
#include <vector>
#include <thread>
#include <condition_variable>

#include <boost/asio.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace phy::serial
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class AsyncSerial
{
public:

   AsyncSerial();
  ~AsyncSerial();


  void open(std::string const & dev_node, size_t const baud_rate);
  void close();

  void transmit(std::vector<uint8_t> const & data);
  std::future<std::vector<uint8_t>> receive(size_t const num_bytes);


private:

  class ReceiveBuffer
  {
  public:
    void push(std::vector<uint8_t> const & received_data);
    std::vector<uint8_t> pop(size_t const num_bytes);
  private:
    std::mutex _mutex;
    std::condition_variable _condition;
    std::vector<uint8_t> _rx_buffer;
  };

  static size_t constexpr RECEIVE_BUFFER_SIZE = 32;

  uint8_t _asio_receive_buffer[RECEIVE_BUFFER_SIZE];
  ReceiveBuffer _receive_buffer;
  boost::asio::io_service _io_service;
  boost::asio::serial_port _serial_port;
  std::thread _io_service_thread;

  void read();
  void readEnd(boost::system::error_code const & error, size_t bytes_transferred);

};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* phy::serial */

#endif /* PHY_SERIAL_ASYNCSERIAL_H_ */
