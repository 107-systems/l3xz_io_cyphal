/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <driver/dynamixel/MX28.h>

#include <cassert>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace driver
{

/**************************************************************************************
 * CONSTANT
 **************************************************************************************/

enum class MX28ControlTable : uint16_t
{
  Torque_Enable   =  64,
  LED             =  65,
  GoalPosition    = 116,
  PresentPosition = 132,
};

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

MX28::MX28(std::shared_ptr<Dynamixel> dyn_ctrl)
: _dyn_ctrl{dyn_ctrl}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

std::optional<Dynamixel::IdVect> MX28::discover()
{
  if (auto [err, id_vect] = _dyn_ctrl->broadcastPing(); err == Dynamixel::Error::None)
    return id_vect;
  else
    return std::nullopt;
}

void MX28::turnLedOn(Dynamixel::IdVect const & id_vect)
{
  assert(id_vect.size() > 0);

  uint8_t led_on = 1;
  Dynamixel::SyncWriteDataVect led_on_data;

  for (auto id : id_vect)
    led_on_data.push_back(std::make_tuple(id, &led_on));

  _dyn_ctrl->syncWrite(static_cast<int>(MX28ControlTable::LED), sizeof(led_on), led_on_data);
}

void MX28::torqueOn(Dynamixel::Id const id)
{
  torqueOn(Dynamixel::IdVect{id});
}

void MX28::torqueOn(Dynamixel::IdVect const & id_vect)
{
  assert(id_vect.size() > 0);

  uint8_t torque_enable_on = 1;
  Dynamixel::SyncWriteDataVect torque_enable_on_data;

  for (auto id : id_vect)
    torque_enable_on_data.push_back(std::make_tuple(id, &torque_enable_on));

  _dyn_ctrl->syncWrite(static_cast<int>(MX28ControlTable::Torque_Enable), sizeof(torque_enable_on), torque_enable_on_data);
}

void MX28::torqueOff(Dynamixel::IdVect const & id_vect)
{
  assert(id_vect.size() > 0);

  uint8_t torque_enable_off = 0;
  Dynamixel::SyncWriteDataVect torque_enable_off_data;

  for (auto id : id_vect)
    torque_enable_off_data.push_back(std::make_tuple(id, &torque_enable_off));

  _dyn_ctrl->syncWrite(static_cast<int>(MX28ControlTable::Torque_Enable), sizeof(torque_enable_off), torque_enable_off_data);
}

void MX28::turnLedOff(Dynamixel::IdVect const & id_vect)
{
  assert(id_vect.size() > 0);

  uint8_t led_off = 0;
  Dynamixel::SyncWriteDataVect led_off_data;

  for (auto id : id_vect)
    led_off_data.push_back(std::make_tuple(id, &led_off));

  _dyn_ctrl->syncWrite(static_cast<int>(MX28ControlTable::LED), sizeof(led_off), led_off_data);
}

std::optional<float> MX28::getAngle(Dynamixel::Id const id)
{
  MX28::AngleDataSet const angle_data_set = getAngle(Dynamixel::IdVect{id});

  if (!angle_data_set.count(id))
    return std::nullopt;

  return angle_data_set.at(id);
}

MX28::AngleDataSet MX28::getAngle(Dynamixel::IdVect const & id_vect)
{
  assert(id_vect.size() > 0);

  AngleDataSet angle_data_set;

  if (auto [err, position_vect] = _dyn_ctrl->syncRead(static_cast<int>(MX28ControlTable::PresentPosition), 4, id_vect); err == Dynamixel::Error::None)
  {
    for (auto [id, position_raw] : position_vect)
      if (position_raw)
      {
        float const position_deg = static_cast<float>(position_raw.value()) * 360.0f / 4096;
        angle_data_set[id] = position_deg;
      }
  }

  return angle_data_set;
}

bool MX28::setAngle(Dynamixel::Id const id, float const angle_deg)
{
  AngleDataSet angle_data_set;
  angle_data_set[id] = angle_deg;
  return setAngle(angle_data_set);
}

bool MX28::setAngle(AngleDataSet const & angle_data_set)
{
  assert(angle_data_set.size() > 0);

  std::vector<uint32_t> position_raw_arr(angle_data_set.size());
  Dynamixel::SyncWriteDataVect sync_write_data_vect;

  size_t arr_idx = 0;
  for (auto [id, position_deg] : angle_data_set)
  {
    uint32_t const position_raw = static_cast<uint32_t>(position_deg * 4096.0f/360.0f);
    position_raw_arr[arr_idx] = position_raw;
    sync_write_data_vect.push_back(std::make_tuple(id, reinterpret_cast<uint8_t *>(position_raw_arr.data() + arr_idx)));
    arr_idx++;
  }

  return (_dyn_ctrl->syncWrite(static_cast<int>(MX28ControlTable::GoalPosition), 4, sync_write_data_vect) == Dynamixel::Error::None);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* driver */
