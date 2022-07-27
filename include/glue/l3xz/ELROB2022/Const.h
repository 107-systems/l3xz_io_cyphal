/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef GLUE_L3XZ_ELROB2022_CONST_H_
#define GLUE_L3XZ_ELROB2022_CONST_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <driver/dynamixel/MX28.h>
#include <driver/dynamixel/Dynamixel.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue::l3xz::ELROB2022
{

/**************************************************************************************
 * CONSTANT
 **************************************************************************************/

static dynamixel::Dynamixel::IdVect const DYNAMIXEL_ID_VECT{1,2,3,4,5,6,7,8};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue::l3xz::ELROB2022 */

#endif /* GLUE_L3XZ_ELROB2022_CONST_H_ */
