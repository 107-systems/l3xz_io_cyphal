/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef COMMON_THREADING_THREADSTATS_H_
#define COMMON_THREADING_THREADSTATS_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <map>
#include <mutex>
#include <thread>
#include <string>
#include <iostream>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::common::threading
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ThreadStats
{
public:

  void add   (std::thread::id const thd_id, std::string const & name);
  void remove(std::thread::id const thd_id);

  friend std::ostream & operator << (std::ostream & os, ThreadStats & stats);

private:

  typedef struct
  {
    std::string name;
  } Data;

  std::map<std::thread::id, Data> _data;
  std::mutex _data_mtx;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::common::threading */

#endif /* COMMON_THREADING_THREADSTATS_H_ */
