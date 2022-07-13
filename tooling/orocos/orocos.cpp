/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <stdio.h>

#include <iostream>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace KDL;
 
/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  KDL::Chain front_right_leg_chain;

  KDL::Vector const front_right_leg_base_coxa(102.0, 216.81, 0.0);
  KDL::Vector const front_right_leg_coxa_femur(20.0, 0.0, -20.0);
  KDL::Vector const front_right_leg_femur_tibia(130.0, 0.0, 90.0);
  KDL::Vector const front_right_leg_tibia_endpoint(25.0, 0.0, -205.0);

  /* Base -> MX-28 Front/Right */
  front_right_leg_chain.addSegment(Segment(Joint(Joint::None), Frame(front_right_leg_base_coxa)));
  /* MX-28 Front/Right z-axis rotation */
  front_right_leg_chain.addSegment(Segment(Joint(Joint::RotZ)));
  /* Coxa axis to femur axis. */
  front_right_leg_chain.addSegment(Segment(Joint(Joint::None), Frame(front_right_leg_coxa_femur)));
  /* Rotation around femur axis. */
  front_right_leg_chain.addSegment(Segment(Joint(Joint::RotY)));
  /* Femur axis to tibia axis. */
  front_right_leg_chain.addSegment(Segment(Joint(Joint::None), Frame(front_right_leg_femur_tibia)));
  /* Rotation around tibia axis. */
  front_right_leg_chain.addSegment(Segment(Joint(Joint::RotY)));
  /* Tibia axis to foot endpoint. */
  front_right_leg_chain.addSegment(Segment(Joint(Joint::None), Frame(front_right_leg_tibia_endpoint)));
  /* Virtual joints to avoid issues with underactuated chains. */
  front_right_leg_chain.addSegment(Segment(Joint(Joint::RotY), Frame(KDL::Vector(0, 0, 0))));
  front_right_leg_chain.addSegment(Segment(Joint(Joint::RotZ), Frame(KDL::Vector(0, 0, 0))));

     /* This is the same. */
//   front_right_leg_chain.addSegment(Segment(Joint(Joint::None), Frame(front_right_leg_coxa)));
//   front_right_leg_chain.addSegment(Segment(Joint(Joint::RotZ)));
//   front_right_leg_chain.addSegment(Segment(Joint(Joint::RotY), Frame(front_right_leg_coxa_femur)));
//   front_right_leg_chain.addSegment(Segment(Joint(Joint::RotY), Frame(front_right_leg_femur_tibia)));
//   front_right_leg_chain.addSegment(Segment(Joint(Joint::None), Frame(front_right_leg_tibia_endpoint)));


  /* Create solver based on kinematic chain. */
  ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(front_right_leg_chain);
 
  /* Create joint array */
  unsigned int nj = front_right_leg_chain.getNrOfJoints();
  KDL::JntArray jointpositions = JntArray(nj);
 
  ChainIkSolverVel_pinv front_right_leg_iksolver_vel(front_right_leg_chain);
  ChainIkSolverPos_NR   front_right_leg_iksolver_pos(front_right_leg_chain, fksolver, front_right_leg_iksolver_vel);

  KDL::JntArray ik_joint_positions = JntArray(nj);

  KDL::Vector ik_target_pos(0, 0, 0);
  std::cout << "Enter the target position X Y Z: " << std::flush;
  std::cin >> ik_target_pos(0) >> ik_target_pos(1) >> ik_target_pos(2);
  std::cout << ik_target_pos << std::endl;

  KDL::Frame ik_target(ik_target_pos);
  if (front_right_leg_iksolver_pos.CartToJnt(jointpositions, ik_target, ik_joint_positions) < 0)
  {
    std::cout << "Error: could not calculate inverse kinematics :(" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "IK results" << std::endl;
  for (size_t r = 0; r < ik_joint_positions.rows(); r++)
     std::cout << ik_joint_positions(r) << std::endl;

  return EXIT_SUCCESS;
}