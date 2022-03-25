/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz/common/threading/ThreadStats.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::common::threading
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void ThreadStats::add(std::thread::id const thd_id, std::string const & name)
{
  std::lock_guard<std::mutex> lock(_data_mtx);
  Data thd_data{name};
  _data[thd_id] = thd_data;
}

void ThreadStats::remove(std::thread::id const thd_id)
{
  std::lock_guard<std::mutex> lock(_data_mtx);
  _data.erase(thd_id);
}

std::ostream & operator << (std::ostream & os, ThreadStats & stats)
{
  std::lock_guard<std::mutex> lock(stats._data_mtx);

  os << "L3XZ Thread Statistics:" << std::endl;
  os << "\tNum Threads: " << stats._data.size() << std::endl;

  for (auto [thd_id, thd_data] : stats._data)
  {
    os << "\t["
       << thd_id
       << "] "
       << thd_data.name
       << std::endl;
  }
  return os;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::common::threading */
