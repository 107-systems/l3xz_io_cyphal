/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ros_cyphal_bridge/graphs/contributors.
 */

#ifndef DYNAMIXEL_MX28_H_
#define DYNAMIXEL_MX28_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <memory>

#include <dynamixel++/Dynamixel++.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace control
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class DynamixelMX28
{
public:

  DynamixelMX28(std::shared_ptr<dynamixelplusplus::Dynamixel> dyn_ctrl);

  std::optional<dynamixelplusplus::Dynamixel::IdVect> discover();


  void turnLedOn (dynamixelplusplus::Dynamixel::IdVect const & id_vect);
  void turnLedOff(dynamixelplusplus::Dynamixel::IdVect const & id_vect);

  void torqueOn (dynamixelplusplus::Dynamixel::Id const id);
  void torqueOn (dynamixelplusplus::Dynamixel::IdVect const & id_vect);
  void torqueOff(dynamixelplusplus::Dynamixel::IdVect const & id_vect);

  typedef std::tuple<dynamixelplusplus::Dynamixel::Id, float> AngleData;
  typedef std::map<dynamixelplusplus::Dynamixel::Id, float> AngleDataSet;

  std::optional<float> getAngle(dynamixelplusplus::Dynamixel::Id const id);
  AngleDataSet         getAngle(dynamixelplusplus::Dynamixel::IdVect const & id_vect);

  bool setAngle(dynamixelplusplus::Dynamixel::Id const id, float const angle_deg);
  bool setAngle(AngleDataSet const & angle_data_set);


private:

  std::shared_ptr<dynamixelplusplus::Dynamixel> _dyn_ctrl;

  enum class ControlTable : uint16_t
  {
    Torque_Enable   =  64,
    LED             =  65,
    GoalPosition    = 116,
    PresentPosition = 132,
  };
};

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef std::shared_ptr<DynamixelMX28> SharedMX28;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* control */

#endif /* DYNAMIXEL_MX28_H_ */
