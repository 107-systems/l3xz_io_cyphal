/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef COMMON_THREADING_THREADSTATS_H_
#define COMMON_THREADING_THREADSTATS_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <list>
#include <mutex>
#include <thread>
#include <string>
#include <iostream>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::threading
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class ThreadStats
{
public:

  void add   (std::string const & thd_name);
  void remove(std::string const & thd_name);

  friend std::ostream & operator << (std::ostream & os, ThreadStats & stats);

private:

  typedef struct
  {
    std::string name;
  } Data;

  std::list<Data> _data;
  std::mutex _data_mtx;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::threading */

#endif /* COMMON_THREADING_THREADSTATS_H_ */
