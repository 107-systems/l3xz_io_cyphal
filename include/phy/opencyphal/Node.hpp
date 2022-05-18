/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
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

class Node;
typedef std::function<void(CanardTransfer const &)> OnTransferReceivedFunc;
typedef std::function<bool(CanardFrame const &)> CanFrameTransmitFunc;

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node
{
public:

  Node(uint8_t const node_id,
       CanFrameTransmitFunc transmit_func,
       SocketCAN & socket_can);

  ~Node();

  /* Must be called regularly from within the application
   * in order to transmit all CAN pushed on the internal
   * stack via publish/request.
   */
  bool transmitCanFrame();


  template <typename T>                     bool subscribe  (OnTransferReceivedFunc func);
  template <typename T>                     bool unsubscribe();

  /* publish/subscribe API for "message" data exchange paradigm */
  template <typename T_MSG>                 bool publish    (T_MSG const & msg);

  /* request/response API for "service" data exchange paradigm */
  template <typename T_RSP>                 bool respond    (T_RSP const & rsp, CanardNodeID const remote_node_id, CanardTransferID const transfer_id);
  template <typename T_REQ, typename T_RSP> bool request    (T_REQ const & req, CanardNodeID const remote_node_id, OnTransferReceivedFunc func);


private:

  static size_t constexpr LIBCANARD_O1HEAP_SIZE = 4096;
  typedef O1Heap<LIBCANARD_O1HEAP_SIZE> O1HeapLibcanard;

  typedef struct
  {
    CanardRxSubscription canard_rx_sub;
    OnTransferReceivedFunc transfer_complete_callback;
  } RxTransferData;

  std::mutex _mtx;
  O1HeapLibcanard _o1heap;
  CanardInstance _canard_ins;
  CanFrameTransmitFunc _transmit_func;
  SocketCAN & _socket_can;
  std::map<CanardPortID, RxTransferData> _rx_transfer_map;
  std::map<CanardPortID, CanardTransferID> _tx_transfer_map;

  std::thread _rx_thread;
  std::atomic<bool> _rx_thread_active;


  static void * o1heap_allocate(CanardInstance * const ins, size_t const amount);
  static void   o1heap_free    (CanardInstance * const ins, void * const pointer);

  CanardTransferID getNextTransferId(CanardPortID const port_id);
  bool             subscribe        (CanardTransferKind const transfer_kind, CanardPortID const port_id, size_t const payload_size_max, OnTransferReceivedFunc func);
  bool             unsubscribe      (CanardTransferKind const transfer_kind, CanardPortID const port_id);
  bool             enqeueTransfer   (CanardNodeID const remote_node_id, CanardTransferKind const transfer_kind, CanardPortID const port_id, size_t const payload_size, void * payload, CanardTransferID const transfer_id);

  void rxThreadFunc();
  void onCanFrameReceived(CanardFrame const & frame);
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
