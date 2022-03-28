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
 * STATIC MEMBER DECLARATION
 **************************************************************************************/

ThreadStats ThreadBase::_stats;

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

ThreadBase::ThreadBase(std::string const & thread_name)
: _thread_name{thread_name}
, _thread_running{false}
, _thd{}
{

}

ThreadBase::~ThreadBase()
{
  stopThread();
}

/**************************************************************************************
 * PROTECTED MEMBER FUNCTIONS
 **************************************************************************************/

void ThreadBase::startThread()
{
  _thread_running = true;
  _stats.add(_thread_name);
  _thd = std::thread([this]{ this->threadFunc(); });
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void ThreadBase::threadFunc()
{
  setup();
  while (_thread_running)
    loop();
}

void ThreadBase::stopThread()
{
  if (_thd.joinable())
  {
    _thread_running = false;
    _thd.join();
  }

  _stats.remove(_thread_name);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::common::threading */
