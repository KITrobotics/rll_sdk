/*
 * This file is part of the Robot Learning Lab SDK
 *
 * Copyright (C) 2020 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RLL_KINEMATICS_INVERSE_KINEMATICS_COEFFICIENTS_H
#define RLL_KINEMATICS_INVERSE_KINEMATICS_COEFFICIENTS_H

#include <rll_kinematics/forward_kinematics.h>

#define RLL_NUM_JOINTS_P 4
#define RLL_NUM_JOINTS_H 2  // excluding the elbow joint, elbow joint is always treated differently

// seed state contains maximum of two past states (to compute velocity and acceleration with finite differences)
#define RLL_MAX_NUM_SEED_TIME_STEPS 2

using RLLKinSeedState = boost::container::static_vector<RLLKinJoints, RLL_MAX_NUM_SEED_TIME_STEPS>;

struct RLLInvKinHelperMatrices
{
  Eigen::Matrix3d as, bs, cs;
  Eigen::Matrix3d aw, bw, cw;
};

class RLLInvKinCoeffs : public RLLKinematicsBase
{
public:
  RLLInvKinCoeffs() = default;

  explicit RLLInvKinCoeffs(RLLKinSeedState seed_state) : seed_state_(std::move(seed_state))
  {
  }

  enum JointType : bool
  {
    PIVOT_JOINT = false,
    HINGE_JOINT = true
  };

  void setSeedState(const RLLKinSeedState& seed_state)
  {
    seed_state_ = seed_state;
  }

  void setHelperMatrices(const RLLInvKinHelperMatrices& hm, const Eigen::Vector3d& xsw_n, const Eigen::Vector3d& xwf_n,
                         double joint_angle_4);
  void setGC(const RLLKinGlobalConfig& config);
  void preparePivotDerivatives();

  void setInitialized()
  {
    initialized_ = true;
    init_error_ = RLLKinMsg::SUCCESS;
  }

  void setInitError(const RLLKinMsg msg)
  {
    init_error_ = msg;
  }

  RLLKinShoulderWristVec const& sw() const
  {
    return sw_;
  }

  RLLKinSeedState const& seedState() const
  {
    return seed_state_;
  }

  bool initialized() const
  {
    return initialized_;
  }

  RLLKinMsg initError() const
  {
    return init_error_;
  }

  double jointAngle4() const
  {
    return joint_angle_4_;
  }

  double jointAngle(JointType type, uint8_t i, double arm_angle) const;
  double jointAnglePivot(uint8_t i, double arm_angle) const;
  double jointAngleHinge(uint8_t i, double arm_angle) const;
  double jointDerivative(JointType type, uint8_t i, double arm_angle, double joint_angle) const;
  double jointDerivativePivot(uint8_t i, double arm_angle) const;
  double jointDerivativeHinge(uint8_t i, double arm_angle, double joint_angle) const;
  bool armAngleForJointLimit(JointType type, uint8_t i, double joint_angle, double* arm_angle_lower,
                             double* arm_angle_upper) const;
  bool armAnglePivot(uint8_t i, double joint_angle, double* arm_angle_lower, double* arm_angle_upper) const;
  bool armAngleHinge(uint8_t i, double joint_angle, double* arm_angle_lower, double* arm_angle_upper) const;

  bool pivotSingularity(uint8_t i, double* value) const;

private:
  // coefficients for pivot joints
  std::array<double, RLL_NUM_JOINTS_P> an_, bn_, cn_;
  std::array<double, RLL_NUM_JOINTS_P> ad_, bd_, cd_;
  std::array<double, RLL_NUM_JOINTS_P> at_, bt_, ct_;
  std::array<double, RLL_NUM_JOINTS_P> gc_p_;

  // coefficients for hinge joints
  std::array<double, RLL_NUM_JOINTS_H> a_, b_, c_;
  std::array<double, RLL_NUM_JOINTS_H> gc_h_;

  RLLKinShoulderWristVec sw_;

  double joint_angle_4_;

  RLLKinSeedState seed_state_;

  bool initialized_ = false;
  RLLKinMsg init_error_ = RLLKinMsg::SUCCESS;
};

#endif  // RLL_KINEMATICS_INVERSE_KINEMATICS_COEFFICIENTS_H
