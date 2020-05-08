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

#ifndef RLL_KINEMATICS_INVERSE_KINEMATICS_H
#define RLL_KINEMATICS_INVERSE_KINEMATICS_H

#include <rll_kinematics/arm_angle_intervals.h>

class RLLInverseKinematics : public RLLForwardKinematics
{
public:
  RLLInverseKinematics() = default;

  // If the absolute value of a hinge joint angle is below this value, the global configs determined by the respective
  // joint are considered close.
  static const double GLOBAL_CONFIG_DISTANCE_TOL;

protected:
  RLLKinMsg ikFixedArmAngleFixedConfig(const RLLKinJoints& seed_state, RLLKinPoseConfig* eef_pose,
                                       RLLKinJoints* joint_angles) const;

  RLLKinMsg computeFeasibleIntervals(RLLInvKinNsIntervals* intervals) const;
  RLLKinMsg jointAnglesFromFixedArmAngle(double arm_angle, const RLLInvKinCoeffs& coeffs,
                                         RLLKinJoints* joint_angles) const;
  RLLKinMsg jointAnglesFromArmAngle(double arm_angle, const RLLInvKinCoeffs& coeffs, RLLKinJoints* joint_angles,
                                    bool assert_limits = false) const;

  // set helper matrices and derivatives, checks if coeffs has already been initialized
  RLLKinMsg setCoeffsWithInitCheck(const RLLKinPoseConfig& eef_pose, RLLInvKinCoeffs* coeffs, double* joint_angle_4,
                                   bool set_derivatives = true) const;

  void determineClosestConfigs(const RLLKinJoints& joint_angles, RLLKinGlobalConfigs* configs) const;
  double mapArmAngleForGC4(const RLLKinGlobalConfig& seed_config, const RLLKinGlobalConfig& selected_config,
                           double seed_arm_angle) const;
  void addRemainingConfigs(size_t it, double dist_from_seed, RLLKinGlobalConfigs* configs) const;

private:
  RLLKinMsg setHelperMatrices(const RLLKinPoseConfig& eef_pose, RLLInvKinCoeffs* coeffs, double* joint_angle_4) const;
};

#endif  // RLL_KINEMATICS_INVERSE_KINEMATICS_H
