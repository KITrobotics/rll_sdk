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

#ifndef RLL_KINEMATICS_REDUNDANCY_RESOLUTION_H
#define RLL_KINEMATICS_REDUNDANCY_RESOLUTION_H

#include <rll_kinematics/inverse_kinematics.h>

struct RLLInvKinOptions
{
  enum Method
  {
    POSITION_RESOLUTION_EXP,  // using the method from Faria et al. based on exponential function
    ARM_ANGLE_FIXED           // desired arm angle is given
  };

  Method method = POSITION_RESOLUTION_EXP;
  bool keep_global_configuration = false;  // allow changes of the global configuration

  // parameters of the position-based resolution
  double position_exp_k = 0.02;
  double position_exp_alpha = 10.0;
};

// TODO(wolfgang): add a method that provides debugging output as feedback, e.g. the computation time
class RLLRedundancyResolution : public RLLInverseKinematics
{
public:
  RLLRedundancyResolution() = default;

  RLLKinMsg ik(const RLLKinJoints& seed_state, RLLKinPoseConfig* ik_pose, RLLKinJoints* solution,
               const RLLInvKinOptions& options) const;

protected:
  // redundancy resolution using fixed arm angle and variable global config
  RLLKinMsg ikFixedArmAngle(const RLLKinJoints& seed_state, RLLKinPoseConfig* ik_pose, RLLKinJoints* solution,
                            const RLLInvKinOptions& options) const;
  // redundancy-resolution using exponential function and fixed global config
  RLLKinMsg ikExpFixedConfig(const RLLKinJoints& seed_state, RLLKinPoseConfig* ik_pose, RLLKinJoints* solution,
                             const RLLInvKinOptions& options) const;

  // get closest solution to seed-state using optimization of arm angle defined in optimize()
  RLLKinMsg ikClosestConfig(const RLLKinJoints& seed_state, RLLKinPoseConfig* ik_pose, RLLKinJoints* solution,
                            const RLLInvKinOptions& options,
                            RLLKinMsg (RLLRedundancyResolution::*optimize)(const RLLInvKinCoeffs&,
                                                                           const RLLInvKinOptions& options,
                                                                           const double, double*, RLLKinJoints*)
                                const) const;
  // redundancy resolution using exponential function
  RLLKinMsg optimizationPositionExp(const RLLInvKinCoeffs& coeffs, const RLLInvKinOptions& options,
                                    double arm_angle_seed, double* arm_angle_new, RLLKinJoints* solution) const;
  RLLKinMsg optimizationFixedArmAngle(const RLLInvKinCoeffs& coeffs, const RLLInvKinOptions& options,
                                      double arm_angle_seed, double* arm_angle_new, RLLKinJoints* solution) const;

  double expResolution(const RLLInvKinOptions& options, double arm_angle_old, double lower_limit,
                       double upper_limit) const;
};

#endif  // RLL_KINEMATICS_REDUNDANCY_RESOLUTION_H
