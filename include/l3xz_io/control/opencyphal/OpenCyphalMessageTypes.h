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

#include <l3xz_io/phy/opencyphal/Types.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace control
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

typedef uavcan::primitive::scalar::Real32_1_0<1002> OpenCyphalFemurAnglePositionDegreeMessage;
typedef uavcan::primitive::scalar::Real32_1_0<1003> OpenCyphalTibiaAnglePositionDegreeMessage;
typedef uavcan::primitive::scalar::Bit_1_0<1004>    OpenCyphalTibiaTipBumperMessage;

typedef reg::udral::service::common::Readiness_0_1<3001>            OpenCyphalOrel20ReadinessMessage;
typedef reg::udral::service::actuator::common::sp::Scalar_0_1<3002> OpenCyphalOrel20SetpointMessage;

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* control */

#endif /* GLUE_OPEN_CYPHAL_MESSAGE_TYPES_H_ */
