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

#ifndef RLL_KINEMATICS_FORWARD_KINEMATICS_H
#define RLL_KINEMATICS_FORWARD_KINEMATICS_H

#include <rll_kinematics/types_utils.h>

using RLLKinLimbs = std::array<double, 4>;

struct RLLKinShoulderWristVec
{
  Eigen::Vector3d xsw_n;  // vector from shoulder to wrist, normalized
  Eigen::Vector3d xwf_n;  // vector from wrist to flange, normalized
};

class RLLForwardKinematics : public RLLKinematicsBase
{
public:
  RLLForwardKinematics() = default;

  // a warning is returned if the distance to a singularity is below this value
  static const double SINGULARITY_CHECK_DISTANCE_TOL;

  RLLKinMsg initialize(const RLLKinLimbs& limb_lengths, const RLLKinJoints& lower_joint_limits,
                       const RLLKinJoints& upper_joint_limits);
  RLLKinMsg fk(const RLLKinJoints& joint_angles, RLLKinPoseConfig* eef_pose) const;

protected:
  RLLKinMsg armAngle(const RLLKinJoints& joint_angles, const RLLKinGlobalConfig& config, double* arm_angle) const;

  // xsw: vector from shoulder to wrist, xwf_n: normalized vector from wrist to flange, lsw: length of xsw
  RLLKinMsg shoulderWristVec(const RLLKinFrame& eef_pose, Eigen::Vector3d* xsw, Eigen::Vector3d* xwf_n,
                             double* lsw) const;

  static double jointAngle1Virtual(const Eigen::Vector3d& xsw);
  double jointAngle2Virtual(const Eigen::Vector3d& xsw, double lsw, const RLLKinGlobalConfig& config) const;
  double jointAngle4(double lsw, const RLLKinGlobalConfig& config) const;

  bool jointLimitsViolated(const RLLKinJoints& joint_angles) const
  {
    return joint_angles.jointLimitsViolated(lower_joint_limits_, upper_joint_limits_);
  }

  static RLLKinMsg checkSingularities(const RLLKinJoints& joint_angles, const RLLKinShoulderWristVec& sw);

  RLLKinJoints const& lowerJointLimits() const
  {
    return lower_joint_limits_;
  }

  RLLKinJoints const& upperJointLimits() const
  {
    return upper_joint_limits_;
  }

  double limbLength(const size_t index) const
  {
    return limb_lengths_[index];
  }

  bool initialized() const
  {
    return initialized_;
  }

private:
  RLLKinMsg armAngle(const RLLKinJoints& joint_angles, const RLLKinGlobalConfig& config, double* arm_angle,
                     RLLKinFrame* flange_pose, RLLKinShoulderWristVec* sw) const;

  // Robot properties
  RLLKinLimbs limb_lengths_;
  RLLKinJoints lower_joint_limits_;
  RLLKinJoints upper_joint_limits_;

  bool initialized_ = false;
};

#endif  // RLL_KINEMATICS_FORWARD_KINEMATICS_H
