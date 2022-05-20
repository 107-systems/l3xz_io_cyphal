/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <common/kinematic/Engine.h>

#include <kdl/frames_io.hpp>

#include <ros/ros.h>
#include <ros/console.h>

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

  _fksolver     = std::unique_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(_leg_chain));
  _iksolver_vel = std::unique_ptr<KDL::ChainIkSolverVel_pinv>     (new KDL::ChainIkSolverVel_pinv(_leg_chain));
  _iksolver_pos = std::unique_ptr<KDL::ChainIkSolverPos_NR>       (new KDL::ChainIkSolverPos_NR(_leg_chain, *_fksolver, *_iksolver_vel));
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

FK_Output Engine::fk_solve(FK_Input const & input)
{
  /* Create joint array. */
  auto const number_joints = _leg_chain.getNrOfJoints();
  KDL::JntArray joint_positions = KDL::JntArray(number_joints);
 
  /* Assign current values to the joint array. */
  joint_positions(0) = input.angle_rad(Joint::Coxa);
  joint_positions(1) = input.angle_rad(Joint::Femur);
  joint_positions(2) = input.angle_rad(Joint::Tibia);

  /* Create the frame that will contain the results */
  KDL::Frame tibia_tip_pos;    
 
  /* Calculate forward position kinematics. */
  if (_fksolver->JntToCart(joint_positions, tibia_tip_pos) < 0)
    throw std::runtime_error("Engine::fk_solve: could not calculate forward kinematics :(");

  std::stringstream msg;
  msg << "FK results" << std::endl << tibia_tip_pos;
  ROS_INFO("%s", msg.str().c_str());
  
  return FK_Output(tibia_tip_pos(3,0),
                   tibia_tip_pos(3,1),
                   tibia_tip_pos(3,2));
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* common::kinematic */
