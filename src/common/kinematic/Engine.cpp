/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <common/kinematic/Engine.h>

#include <kdl/frames_io.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace common::kinematic
{

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static KDL::Vector const COXA_TO_FEMUR ( 20.0, 0.0,  -20.0);
static KDL::Vector const FEMUR_TO_TIBIA(130.0, 0.0,   90.0);
static KDL::Vector const TIBIA_TO_TIP  ( 25.0, 0.0, -205.0);

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Engine::Engine()
{
  /* MX-28 Front/Right z-axis rotation */
  _leg_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
  /* Coxa axis to femur axis. */
  _leg_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(COXA_TO_FEMUR)));
  /* Rotation around femur axis. */
  _leg_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
  /* Femur axis to tibia axis. */
  _leg_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(FEMUR_TO_TIBIA)));
  /* Rotation around tibia axis. */
  _leg_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
  /* Tibia axis to foot endpoint. */
  _leg_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame(TIBIA_TO_TIP)));
  /* Virtual joints to avoid issues with underactuated chains.
   * Rotation about X is not needed if the X orientation setpoint is always zero.
   */
  _leg_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame(KDL::Vector::Zero())));
  _leg_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame(KDL::Vector::Zero())));

  _fksolver     = std::unique_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(_leg_chain));
  _iksolver_vel = std::unique_ptr<KDL::ChainIkSolverVel_pinv>     (new KDL::ChainIkSolverVel_pinv(_leg_chain));
  _iksolver_pos = std::unique_ptr<KDL::ChainIkSolverPos_NR>       (new KDL::ChainIkSolverPos_NR(_leg_chain, *_fksolver, *_iksolver_vel, /* maxiter = */ 200, /* eps = */ 1e-1));
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

std::optional<FK_Output> Engine::fk_solve(FK_Input const & fk_input) const
{
  /* Create the frame that will contain the results */
  KDL::Frame tibia_tip_frame;
 
  /* Calculate forward position kinematics. */
  auto const number_joints = _leg_chain.getNrOfJoints();
  KDL::JntArray joint_positions_in  = KDL::JntArray(number_joints);  // Includes the virtual segments.
  const auto joint_positions_real = fk_input.joint_positions();
  for (auto i = 0U; i < joint_positions_real.rows(); i++)
  {
    joint_positions_in(i) = joint_positions_real(i);
  }
  if (auto const rc = _fksolver->JntToCart(joint_positions_in, tibia_tip_frame); rc < 0)
  {
    printf("[ERROR] Engine::fk_solve, ChainFkSolverPos_recursive::JntToCart failed with %d", rc);
    return std::nullopt;
  }

  std::stringstream msg;
  msg << "FK results" << std::endl << tibia_tip_frame;
  printf("[INFO] %s", msg.str().c_str());
  
  FK_Output const fk_output(tibia_tip_frame);
  printf("[INFO] %s", fk_output.toStr().c_str());
  return fk_output;
}

std::optional<IK_Output> Engine::ik_solve(IK_Input const & ik_input) const
{
  /* Create joint array. */
  auto const number_joints = _leg_chain.getNrOfJoints();
  KDL::JntArray joint_positions_out = KDL::JntArray(number_joints);
  KDL::JntArray joint_positions_in  = KDL::JntArray(number_joints);  // Includes the virtual segments.
  const auto joint_positions_real = ik_input.joint_positions();
  for (auto i = 0U; i < joint_positions_real.rows(); i++)
  {
    joint_positions_in(i) = joint_positions_real(i);
  }

  /* Perform IK calculation. */
  if (auto const rc = _iksolver_pos->CartToJnt(joint_positions_in, ik_input.tibia_tip_frame(), joint_positions_out); rc < 0)
  {
    printf("[ERROR] Engine::ik_solve, ChainIkSolverPos_NR::CartToJnt failed with %d", rc);
    return std::nullopt;
  }

  /* Print/return results. */
  std::stringstream msg;
  msg << "IK results" << std::endl;
  for (size_t r = 0; r < joint_positions_out.rows(); r++)
    msg << joint_positions_out(r) << std::endl;
  printf("[INFO] %s", msg.str().c_str());

  IK_Output const ik_output(
    +joint_positions_out(0),
    -joint_positions_out(1),
    +joint_positions_out(2)
  );
  printf("[INFO] %s", ik_output.toStr().c_str());
  return ik_output;
}


/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::kinematic */
