/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef PHY_OPENCYPHAL_O1HEAP_HPP_
#define PHY_OPENCYPHAL_O1HEAP_HPP_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <o1heap.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace phy::opencyphal
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

template <size_t HEAP_SIZE>
class O1Heap
{
public:
  O1Heap() : _o1heap_ins{o1heapInit(_base, HEAP_SIZE)}
  { }

  inline void * allocate(size_t const amount) { return o1heapAllocate(_o1heap_ins, amount); }
  inline void   free    (void * const pointer) { return o1heapFree(_o1heap_ins, pointer); }


private:
  uint8_t _base[HEAP_SIZE] __attribute__ ((aligned (O1HEAP_ALIGNMENT)));
  O1HeapInstance * _o1heap_ins;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* phy::opencyphal */

#endif /* PHY_OPENCYPHAL_O1HEAP_HPP_ */
