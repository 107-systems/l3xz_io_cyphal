/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef DYNAMIXEL_MX28_H_
#define DYNAMIXEL_MX28_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <memory>

#include <l3xz/driver/dynamixel/Dynamixel.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace dynamixel
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class MX28
{
public:

  MX28(std::unique_ptr<Dynamixel> dyn_ctrl);

  std::optional<IdVect> discover();


  void turnLedOn (IdVect const & id_vect);
  void turnLedOff(IdVect const & id_vect);

  void torqueOn (uint8_t const id);
  void torqueOn (IdVect const & id_vect);
  void torqueOff(IdVect const & id_vect);

  typedef std::tuple<uint8_t, float> AngleData;
  typedef std::vector<AngleData> AngleDataVect;

  std::optional<AngleData> getAngle(uint8_t const id);
  AngleDataVect            getAngle(IdVect const & id_vect);

  bool setAngle(AngleData const & angle_data);
  bool setAngle(AngleDataVect const & angle_data_vect);


private:

  std::unique_ptr<Dynamixel> _dyn_ctrl;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* dynamixel */

#endif /* DYNAMIXEL_MX28_H_ */
