/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <phy/opencyphal/Node.hpp>

#include <cstring>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace phy::opencyphal
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Node::Node(uint8_t const node_id,
           SocketCAN & socket_can)
: _mtx{}
, _canard_ins{canardInit(Node::o1heap_allocate, Node::o1heap_free)}
, _socket_can{socket_can}
, _rx_thread{}
, _rx_thread_active{false}
, _tx_thread{}
, _tx_thread_active{false}
{
  _canard_ins.node_id = node_id;
  _canard_ins.mtu_bytes = CANARD_MTU_CAN_CLASSIC;
  _canard_ins.user_reference = reinterpret_cast<void *>(&_o1heap);

  _rx_thread = std::thread([this]() { this->rxThreadFunc(); });
  _tx_thread = std::thread([this]() { this->txThreadFunc(); });
}

Node::~Node()
{
  _rx_thread_active = false;
  _rx_thread.join();

  _tx_thread_active = false;
  _tx_thread.join();
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void * Node::o1heap_allocate(CanardInstance * const ins, size_t const amount)
{
  O1HeapLibcanard * o1heap = reinterpret_cast<O1HeapLibcanard*>(ins->user_reference);
  return o1heap->allocate(amount);
}

void Node::o1heap_free(CanardInstance * const ins, void * const pointer)
{
  O1HeapLibcanard * o1heap = reinterpret_cast<O1HeapLibcanard*>(ins->user_reference);
  o1heap->free(pointer);
}

CanardTransferID Node::getNextTransferId(CanardPortID const port_id)
{
  CanardTransferID const next_transfer_id = (_tx_transfer_map.count(port_id) > 0) ? ((_tx_transfer_map[port_id] + 1) % CANARD_TRANSFER_ID_MAX) : 0;
  _tx_transfer_map[port_id] = next_transfer_id;
  return next_transfer_id;
}

bool Node::subscribe(CanardTransferKind const transfer_kind, CanardPortID const port_id, size_t const payload_size_max, OnTransferReceivedFunc func)
{
  _rx_transfer_map[port_id].transfer_complete_callback = func;
  int8_t const result = canardRxSubscribe(&_canard_ins,
                                          transfer_kind,
                                          port_id,
                                          payload_size_max,
                                          CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                                          &(_rx_transfer_map[port_id].canard_rx_sub));
  bool const success = (result >= 0);
  return success;
}

bool Node::unsubscribe(CanardTransferKind const transfer_kind, CanardPortID const port_id)
{
  int8_t const result = canardRxUnsubscribe(&_canard_ins,
                                            transfer_kind,
                                            port_id);

  /* Remove CanardRxSubscription instance from internal list since the
   * structure is no longed needed.
   */
  _rx_transfer_map.erase(port_id);

  bool const success = (result >= 0);
  return success;
}

bool Node::enqeueTransfer(CanardNodeID const remote_node_id, CanardTransferKind const transfer_kind, CanardPortID const port_id, size_t const payload_size, void * payload, CanardTransferID const transfer_id)
{
  CanardTransfer const transfer =
  {
    /* .timestamp_usec = */ 0, /* No deadline on transmission */
    /* .priority       = */ CanardPriorityNominal,
    /* .transfer_kind  = */ transfer_kind,
    /* .port_id        = */ port_id,
    /* .remote_node_id = */ remote_node_id,
    /* .transfer_id    = */ transfer_id,
    /* .payload_size   = */ payload_size,
    /* .payload        = */ payload,
  };

  /* Serialize transfer into a series of CAN frames */
  int32_t result = canardTxPush(&_canard_ins, &transfer);
  bool const success = (result >= 0);
  return success;
}

void Node::rxThreadFunc()
{
  _rx_thread_active = true;

  printf("[INFO] Node::rxThreadFunc starting  ...");

  while (_rx_thread_active)
  {
    CanardFrame rx_frame;
    uint8_t payload_buffer[64] = {0};

    int16_t const rc = _socket_can.pop(&rx_frame, sizeof(payload_buffer), payload_buffer, CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, nullptr);
    if (rc < 0) {
      printf("[ERROR] socketcanPop failed with error %s ", strerror(abs(rc)));
    }
    else if (rc == 0) {
      printf("[INFO] socketcanPop timeout while receiving.");
    }
    else {
      onCanFrameReceived(rx_frame);
    }
  }

  printf("[INFO] Node::rxThreadFunc stopping  ...");
}

void Node::onCanFrameReceived(CanardFrame const & frame)
{
  std::lock_guard<std::mutex> lock(_mtx);

  CanardTransfer transfer;
  int8_t const result = canardRxAccept(&_canard_ins,
                                       &frame,
                                       0,
                                       &transfer);

  if(result == 1)
  {
    if (_rx_transfer_map.count(transfer.port_id) > 0)
    {
      OnTransferReceivedFunc transfer_received_func = _rx_transfer_map[transfer.port_id].transfer_complete_callback;

      if (transfer.transfer_kind == CanardTransferKindResponse) {
        if ((_tx_transfer_map.count(transfer.port_id) > 0) && (_tx_transfer_map[transfer.port_id] == transfer.transfer_id))
        {
          transfer_received_func(transfer);
          unsubscribe(CanardTransferKindResponse, transfer.port_id);
        }
      }
      else
        transfer_received_func(transfer);
    }
    _o1heap.free(const_cast<void *>(transfer.payload));
  }
}

void Node::txThreadFunc()
{
  _tx_thread_active = true;

  printf("[INFO] Node::txThreadFunc starting  ...");

  while (_tx_thread_active)
  {
    std::lock_guard<std::mutex> lock(_mtx);

    /* Obtain CAN frame of data yet to be transferred. */
    CanardFrame const * tx_frame = canardTxPeek(&_canard_ins);

    if (tx_frame)
    {
      /* Transmit CAN frame. */
      if (int16_t const rc = _socket_can.push(tx_frame, CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC); rc <= 0) {
        printf("[ERROR] socketcanPush failed with error %d", abs(rc));
      }
      else
      {
        /* Remove from both canard and o1heap. */
        canardTxPop(&_canard_ins);
        _o1heap.free((void *)(tx_frame));
      }
    }
    /* Nothing to transmit. */
    else
    {
      std::this_thread::yield();
    }
  }

  printf("[INFO] Node::txThreadFunc stopping  ...");
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* phy::opencyphal */
