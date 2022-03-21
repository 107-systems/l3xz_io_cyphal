/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz/common/threading/ThreadBase.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::common::threading
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

ThreadBase::ThreadBase(std::string const & thread_name)
: _terminate_thread{false}
, _thd{[this]{ threadFunc(); }}
{
  _stats.add(std::this_thread::get_id(), thread_name);
}

ThreadBase::~ThreadBase()
{
  _stats.remove(std::this_thread::get_id());
  _terminate_thread = true;
  _thd.join();
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

std::ostream & operator << (std::ostream & os, ThreadBase const & thd_base)
{
  os << thd_base._stats;
  return os;
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void ThreadBase::threadFunc()
{
  setup();
  while (!_terminate_thread)
    loop();
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::common::threading */
