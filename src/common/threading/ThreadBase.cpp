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
  _thd = std::thread([this]{ this->threadFunc(); });
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void ThreadBase::threadFunc()
{
  _thread_running = true;
  std::cout << "threadFunc - 1" << std::endl;
  _stats.add(std::this_thread::get_id(), _thread_name);
  std::cout << "threadFunc - 2" << std::endl;
  setup();
  std::cout << "threadFunc - 3" << std::endl;
  while (_thread_running)
    loop();
}

void ThreadBase::stopThread()
{
  _stats.remove(std::this_thread::get_id());

  if (_thd.joinable())
  {
    _thread_running = false;
    _thd.join();
  }
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::common::threading */
