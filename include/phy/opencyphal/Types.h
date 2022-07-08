/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2020 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-UAVCAN/graphs/contributors.
 */

#ifndef PHY_OPENCYPHAL_TYPES_H_
#define PHY_OPENCYPHAL_TYPES_H_

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

//namespace phy::opencyphal
//{

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

/* uavcan/node ************************************************************************/
#include "wrappers/uavcan/node/Health_1_0.hpp"
#include "wrappers/uavcan/node/Heartbeat_1_0.hpp"
#include "wrappers/uavcan/node/Mode_1_0.hpp"

/* /uavcan/primitive/scalar ***********************************************************/
#include "wrappers/uavcan/primitive/scalar/Bit_1_0.hpp"
#include "wrappers/uavcan/primitive/scalar/Integer8_1_0.hpp"
#include "wrappers/uavcan/primitive/scalar/Integer16_1_0.hpp"
#include "wrappers/uavcan/primitive/scalar/Integer32_1_0.hpp"
#include "wrappers/uavcan/primitive/scalar/Integer64_1_0.hpp"
#include "wrappers/uavcan/primitive/scalar/Natural8_1_0.hpp"
#include "wrappers/uavcan/primitive/scalar/Natural16_1_0.hpp"
#include "wrappers/uavcan/primitive/scalar/Natural32_1_0.hpp"
#include "wrappers/uavcan/primitive/scalar/Natural64_1_0.hpp"
#include "wrappers/uavcan/primitive/scalar/Real16_1_0.hpp"
#include "wrappers/uavcan/primitive/scalar/Real32_1_0.hpp"
#include "wrappers/uavcan/primitive/scalar/Real64_1_0.hpp"

#include "wrappers/reg/udral/service/actuator/common/sp/Vector31_0_1.hpp"

#include "wrappers/reg/udral/service/common/Readiness_0_1.hpp"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

//} /* phy::opencyphal */

#endif /* PHY_OPENCYPHAL_TYPES_H_ */
