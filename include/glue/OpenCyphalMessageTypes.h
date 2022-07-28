/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

#ifndef GLUE_OPEN_CYPHAL_MESSAGE_TYPES_H_
#define GLUE_OPEN_CYPHAL_MESSAGE_TYPES_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <phy/opencyphal/Types.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace glue
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef uavcan::primitive::scalar::Real32_1_0<1002> OpenCyphalFemurAnglePositionDegreeMessage;
typedef uavcan::primitive::scalar::Real32_1_0<1003> OpenCyphalTibiaAnglePositionDegreeMessage;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* glue */

#endif /* GLUE_OPEN_CYPHAL_MESSAGE_TYPES_H_ */
