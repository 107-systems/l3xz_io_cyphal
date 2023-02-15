/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ros_cyphal_bridge/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_ros_cyphal_bridge/control/dynamixel/DynamixelMX28.h>

#include <cassert>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace control
{

using namespace dynamixelplusplus;

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

DynamixelMX28::DynamixelMX28(std::shared_ptr<Dynamixel> dyn_ctrl)
: _dyn_ctrl{dyn_ctrl}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

std::optional<Dynamixel::IdVect> DynamixelMX28::discover()
{
  if (auto [err, id_vect] = _dyn_ctrl->broadcastPing(); err == Dynamixel::Error::None)
    return id_vect;
  else
    return std::nullopt;
}

void DynamixelMX28::turnLedOn(Dynamixel::IdVect const & id_vect)
{
  assert(id_vect.size() > 0);

  std::map<Dynamixel::Id, uint8_t> led_on_data_map;

  for (auto id : id_vect)
    led_on_data_map[id] = 1;

  _dyn_ctrl->syncWrite(static_cast<int>(ControlTable::LED), led_on_data_map);
}

void DynamixelMX28::torqueOn(Dynamixel::Id const id)
{
  torqueOn(Dynamixel::IdVect{id});
}

void DynamixelMX28::torqueOn(Dynamixel::IdVect const & id_vect)
{
  assert(id_vect.size() > 0);

  std::map<Dynamixel::Id, uint8_t> torque_enable_on_map;

  for (auto id : id_vect)
    torque_enable_on_map[id] = 1;

  _dyn_ctrl->syncWrite(static_cast<int>(ControlTable::Torque_Enable), torque_enable_on_map);
}

void DynamixelMX28::torqueOff(Dynamixel::IdVect const & id_vect)
{
  assert(id_vect.size() > 0);

  std::map<Dynamixel::Id, uint8_t> torque_enable_off_map;

  for (auto id : id_vect)
    torque_enable_off_map[id] = 0;

  _dyn_ctrl->syncWrite(static_cast<int>(ControlTable::Torque_Enable), torque_enable_off_map);
}

void DynamixelMX28::turnLedOff(Dynamixel::IdVect const & id_vect)
{
  assert(id_vect.size() > 0);

  std::map<Dynamixel::Id, uint8_t> led_off_data_map;

  for (auto id : id_vect)
    led_off_data_map[id] = 0;

  _dyn_ctrl->syncWrite(static_cast<int>(ControlTable::LED), led_off_data_map);
}

std::optional<float> DynamixelMX28::getAngle(Dynamixel::Id const id)
{
  DynamixelMX28::AngleDataSet const angle_data_set = getAngle(Dynamixel::IdVect{id});

  if (!angle_data_set.count(id))
    return std::nullopt;

  return angle_data_set.at(id);
}

DynamixelMX28::AngleDataSet DynamixelMX28::getAngle(Dynamixel::IdVect const & id_vect)
{
  AngleDataSet angle_data_set;

  /* Return empty angle data set. */
  if(!id_vect.size())
    return angle_data_set;

  std::map<Dynamixel::Id, uint32_t> position_map;
  if (auto err = _dyn_ctrl->syncRead(static_cast<int>(ControlTable::PresentPosition), id_vect, position_map); err != Dynamixel::Error::None)
    return angle_data_set;

  for (auto [id, position_raw] : position_map)
  {
    float const position_deg = static_cast<float>(position_raw) * 360.0f / 4096;
    angle_data_set[id] = position_deg;
  }

  return angle_data_set;
}

bool DynamixelMX28::setAngle(Dynamixel::Id const id, float const angle_deg)
{
  AngleDataSet angle_data_set;
  angle_data_set[id] = angle_deg;
  return setAngle(angle_data_set);
}

bool DynamixelMX28::setAngle(AngleDataSet const & angle_data_set)
{
  if(!angle_data_set.size())
    return false;

  std::map<Dynamixel::Id, uint32_t> goal_position_data_map;
  for (auto [id, position_deg] : angle_data_set)
  {
    uint32_t const position_raw = static_cast<uint32_t>(position_deg * 4096.0f/360.0f);
    goal_position_data_map[id] = position_raw;
  }

  return (_dyn_ctrl->syncWrite(static_cast<int>(ControlTable::GoalPosition), goal_position_data_map) == Dynamixel::Error::None);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* control */
