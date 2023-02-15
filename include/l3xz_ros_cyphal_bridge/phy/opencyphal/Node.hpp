/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ros_cyphal_bridge/graphs/contributors.
 */

#ifndef PHY_OPENCYPHAL_NODE_HPP_
#define PHY_OPENCYPHAL_NODE_HPP_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <stdint.h>

#include <map>
#include <mutex>
#include <thread>
#include <memory>
#include <atomic>
#include <functional>

#include <canard.h>

#include <rclcpp/rclcpp.hpp>

#include "O1Heap.hpp"
#include "SocketCAN.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace phy::opencyphal
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::function<void(CanardRxTransfer const &)> OnTransferReceivedFunc;

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node
{
public:

  static size_t       constexpr DEFAULT_O1HEAP_SIZE   = 32768;
  static size_t       constexpr DEFAULT_TX_QUEUE_SIZE = 100;
  static size_t       constexpr DEFAULT_MTU_SIZE      = CANARD_MTU_CAN_CLASSIC;
  static CanardNodeID constexpr DEFAULT_NODE_ID       = 42;

  Node(SocketCAN & socket_can, rclcpp::Logger const logger);

  ~Node();


  template <typename T>                     bool subscribe  (OnTransferReceivedFunc func);
  template <typename T>                     bool unsubscribe();

  /* publish/subscribe API for "message" data exchange paradigm */
  template <typename T_MSG>                 bool publish    (T_MSG const & msg, CanardNodeID const remote_node_id);
  template <typename T_MSG>                 bool publish    (T_MSG const & msg) { return publish(msg, CANARD_NODE_ID_UNSET); }

  /* request/response API for "service" data exchange paradigm */
  template <typename T_RSP>                 bool respond    (T_RSP const & rsp, CanardNodeID const remote_node_id, CanardTransferID const transfer_id);
  template <typename T_REQ, typename T_RSP> bool request    (T_REQ const & req, CanardNodeID const remote_node_id, OnTransferReceivedFunc func);


private:

  typedef O1Heap<DEFAULT_O1HEAP_SIZE> O1HeapLibcanard;

  typedef struct
  {
    CanardRxSubscription canard_rx_sub;
    OnTransferReceivedFunc transfer_complete_callback;
  } RxTransferData;

  std::mutex _mtx;
  O1HeapLibcanard _o1heap_hdl;
  CanardInstance _canard_hdl;
  CanardTxQueue _canard_tx_queue;
  SocketCAN & _socket_can;
  rclcpp::Logger const _logger;
  std::map<CanardPortID, RxTransferData> _rx_transfer_map;
  std::map<CanardPortID, CanardTransferID> _tx_transfer_map;

  std::thread _rx_thread;
  std::atomic<bool> _rx_thread_active;

  std::thread _tx_thread;
  std::atomic<bool> _tx_thread_active;


  static void * o1heap_allocate(CanardInstance * const ins, size_t const amount);
  static void   o1heap_free    (CanardInstance * const ins, void * const pointer);

  CanardTransferID getNextTransferId(CanardPortID const port_id);
  bool             subscribe        (CanardTransferKind const transfer_kind, CanardPortID const port_id, size_t const payload_size_max, OnTransferReceivedFunc func);
  bool             unsubscribe      (CanardTransferKind const transfer_kind, CanardPortID const port_id);
  bool             enqeueTransfer   (CanardNodeID const remote_node_id, CanardTransferKind const transfer_kind, CanardPortID const port_id, size_t const payload_size, void * payload, CanardTransferID const transfer_id);

  void rxThreadFunc();
  void onCanFrameReceived(CanardFrame const & frame, CanardMicrosecond const rx_timestamp_us);
  void txThreadFunc();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* phy::opencyphal */

/**************************************************************************************
 * TEMPLATE SOURCE FILE
 **************************************************************************************/

#include "Node.ipp"

#endif /* PHY_OPENCYPHAL_NODE_HPP_ */
