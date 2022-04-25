/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <common/threading/ThreadStats.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::threading
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void ThreadStats::add(std::string const & thd_name)
{
  std::lock_guard<std::mutex> lock(_data_mtx);
  Data thd_data{thd_name};
  _data.push_back(thd_data);
}

void ThreadStats::remove(std::string const & thd_name)
{
  std::lock_guard<std::mutex> lock(_data_mtx);
  _data.remove_if([thd_name](Data const & d) { return (d.name == thd_name); });
}

std::ostream & operator << (std::ostream & os, ThreadStats & stats)
{
  std::lock_guard<std::mutex> lock(stats._data_mtx);

  os << "L3XZ Thread Statistics:" << std::endl;
  os << "\tNum Threads: " << stats._data.size() << std::endl;

  for (auto thd_data : stats._data)
  {
    os << "\t["
       << thd_data.name
       << "] "
       << std::endl;
  }
  return os;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::threading */
