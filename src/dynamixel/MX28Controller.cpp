/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz/dynamixel/MX28Controller.h>

#include <cassert>

#include <ros/console.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace dynamixel
{

/**************************************************************************************
 * CONSTANT
 **************************************************************************************/

enum class MX28ControlTable : uint16_t
{
  LED             =  65,
  PresentPosition = 132,
};

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

MX28Controller::MX28Controller(std::unique_ptr<DynamixelController> dyn_ctrl)
: _dyn_ctrl{std::move(dyn_ctrl)}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

std::optional<IdVect> MX28Controller::discover()
{
  if (auto [err, id_vect] = _dyn_ctrl->broadcastPing(); err == Error::None)
    return id_vect;
  else
    return std::nullopt;
}

void MX28Controller::turnLedOn(IdVect const & id_vect)
{
  assert(id_vect.size() > 0);

  uint8_t led_on = 1;
  SyncWriteDataVect led_on_data;

  for (auto id : id_vect)
    led_on_data.push_back(std::make_tuple(id, &led_on));

  _dyn_ctrl->syncWrite(static_cast<int>(MX28ControlTable::LED), sizeof(led_on), led_on_data);
}

void MX28Controller::turnLedOff(IdVect const & id_vect)
{
  assert(id_vect.size() > 0);

  uint8_t led_off = 0;
  SyncWriteDataVect led_off_data;

  for (auto id : id_vect)
    led_off_data.push_back(std::make_tuple(id, &led_off));

  _dyn_ctrl->syncWrite(static_cast<int>(MX28ControlTable::LED), sizeof(led_off), led_off_data);
}

MX28Controller::AngleDataVect MX28Controller::getCurrentPosition(IdVect const & id_vect)
{
  assert(id_vect.size() > 0);

  AngleDataVect angle_data_vect;

  if (auto [err, position_vect] = _dyn_ctrl->syncRead(static_cast<int>(MX28ControlTable::PresentPosition), 4, id_vect); err == Error::None)
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

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* dynamixel */
