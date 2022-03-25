/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef COMMON_THREADING_THREADBASE_H_
#define COMMON_THREADING_THREADBASE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <string>
#include <thread>
#include <atomic>
#include <iostream>

#include "ThreadStats.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::common::threading
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ThreadBase
{
public:

   ThreadBase(std::string const & thread_name);
  ~ThreadBase();

  static void stats(std::ostream & out) {
    out << _stats << std::endl;
  }

protected:

  void startThread();

  virtual void setup() = 0;
  virtual void loop () = 0;

private:

  std::string const & _thread_name;
  std::atomic<bool> _thread_running;
  std::thread _thd;
  static ThreadStats _stats;

  void threadFunc();
  void stopThread();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::common::threading */

#endif /* COMMON_THREADING_THREADBASE_H_ */
