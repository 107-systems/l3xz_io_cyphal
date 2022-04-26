/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

#ifndef DYNAMIXEL_MX28_H_
#define DYNAMIXEL_MX28_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <memory>

#include <driver/dynamixel/Dynamixel.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace driver
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class MX28
{
public:

  MX28(std::shared_ptr<Dynamixel> dyn_ctrl);

  std::optional<Dynamixel::IdVect> discover();


  void turnLedOn (Dynamixel::IdVect const & id_vect);
  void turnLedOff(Dynamixel::IdVect const & id_vect);

  void torqueOn (Dynamixel::Id const id);
  void torqueOn (Dynamixel::IdVect const & id_vect);
  void torqueOff(Dynamixel::IdVect const & id_vect);

  typedef std::tuple<Dynamixel::Id, float> AngleData;
  typedef std::map<Dynamixel::Id, float> AngleDataSet;

  std::optional<float> getAngle(Dynamixel::Id const id);
  AngleDataSet         getAngle(Dynamixel::IdVect const & id_vect);

  bool setAngle(Dynamixel::Id const id, float const angle_deg);
  bool setAngle(AngleDataSet const & angle_data_set);


private:

  std::shared_ptr<Dynamixel> _dyn_ctrl;
};

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::shared_ptr<MX28> SharedMX28;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* driver */

#endif /* DYNAMIXEL_MX28_H_ */
