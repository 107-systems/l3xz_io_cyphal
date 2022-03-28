/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz/driver/dynamixel/MX28.h>

#include <cassert>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::driver
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
  MX28::AngleDataVect const angle_data_vect = getAngle(Dynamixel::IdVect{id});

  if (!angle_data_vect.size())
    return std::nullopt;

  auto [rx_id, rx_angle_deg] = angle_data_vect.front();
  return rx_angle_deg;   
}

MX28::AngleDataVect MX28::getAngle(Dynamixel::IdVect const & id_vect)
{
  assert(id_vect.size() > 0);

  AngleDataVect angle_data_vect;

  if (auto [err, position_vect] = _dyn_ctrl->syncRead(static_cast<int>(MX28ControlTable::PresentPosition), 4, id_vect); err == Dynamixel::Error::None)
  {
    for (auto [id, position_raw] : position_vect)
      if (position_raw)
      {
        float const position_deg = static_cast<float>(position_raw.value()) * 360.0f / 4096;
        angle_data_vect.push_back(std::make_tuple(id, position_deg));
      }
  }

  return angle_data_vect;
}

bool MX28::setAngle(AngleData const & angle_data)
{
  return setAngle(AngleDataVect{angle_data});
}

bool MX28::setAngle(AngleDataVect const & angle_data_vect)
{
  assert(angle_data_vect.size() > 0);

  std::vector<uint32_t> position_raw_vect;
  Dynamixel::SyncWriteDataVect sync_write_data_vect;

  for (auto [id, position_deg] : angle_data_vect)
  {
    position_raw_vect.push_back(static_cast<uint32_t>(position_deg * 4096.0f/360.0f));
    uint8_t * position_raw_data_ptr = reinterpret_cast<uint8_t *>(&position_raw_vect.back());
    sync_write_data_vect.push_back(std::make_tuple(id, position_raw_data_ptr));
  }

  return (_dyn_ctrl->syncWrite(static_cast<int>(MX28ControlTable::GoalPosition), 4, sync_write_data_vect) == Dynamixel::Error::None);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::driver */
