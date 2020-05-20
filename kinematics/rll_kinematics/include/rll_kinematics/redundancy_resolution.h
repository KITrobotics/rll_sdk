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

  enum GlobalConfigurationControl
  {
    // the nearest config is the first result in the solution vector, more solutions are returned if the seed state is
    // close to other global configs
    SELECT_NEAREST_GLOBAL_CONFIG,
    // only one solution with the same global config as the seed state is returned
    KEEP_CURRENT_GLOBAL_CONFIG,
    // All possible solutions in the different configurations using the specified redundancy resolution are returned,
    // maximum is 8 solutions. The solutions are sorted depending on their distance to the seed state.
    RETURN_ALL_GLOBAL_CONFIGS
  };

  Method method = POSITION_RESOLUTION_EXP;
  GlobalConfigurationControl global_configuration_mode = SELECT_NEAREST_GLOBAL_CONFIG;

  // parameters of the position-based resolution
  double position_exp_k = 0.02;
  double position_exp_alpha = 10.0;
};

// TODO(wolfgang): add a method that provides debugging output as feedback, e.g. the computation time
class RLLRedundancyResolution : public RLLInverseKinematics
{
public:
  RLLRedundancyResolution() = default;

  RLLKinMsg ik(const RLLKinJoints& seed_state, RLLKinPoseConfig* ik_pose, RLLKinSolutions* solution,
               const RLLInvKinOptions& options) const;

protected:
  class ClosestConfigsSolution
  {
  public:
    ClosestConfigsSolution() = default;
    ClosestConfigsSolution(RLLKinJoints solution, double dist_from_seed, RLLKinPoseConfig pose, RLLKinMsg result)
      : solution_(solution), dist_from_seed_(dist_from_seed), pose_(std::move(pose)), result_(result)
    {
    }

    RLLKinJoints solution() const
    {
      return solution_;
    }

    RLLKinPoseConfig pose() const
    {
      return pose_;
    }

    RLLKinMsg result() const
    {
      return result_;
    }

    bool operator<(const ClosestConfigsSolution& rhs) const
    {
      return dist_from_seed_ < rhs.dist_from_seed_;
    }

  private:
    RLLKinJoints solution_;
    double dist_from_seed_;
    RLLKinPoseConfig pose_;
    RLLKinMsg result_;
  };

  // redundancy resolution using fixed arm angle and variable global config
  RLLKinMsg ikFixedArmAngle(const RLLKinJoints& seed_state, RLLKinPoseConfig* ik_pose, RLLKinSolutions* solution,
                            const RLLInvKinOptions& options) const;
  // redundancy-resolution using exponential function and fixed global config
  RLLKinMsg ikExpFixedConfig(const RLLKinJoints& seed_state, RLLKinPoseConfig* ik_pose, RLLKinSolutions* solution,
                             const RLLInvKinOptions& options) const;

  // get closest solution to seed-state using optimization of arm angle defined in optimize()
  RLLKinMsg ikClosestConfig(const RLLKinJoints& seed_state, RLLKinPoseConfig* ik_pose, RLLKinSolutions* solution,
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

  static double expResolution(const RLLInvKinOptions& options, double arm_angle_old, double lower_limit,
                              double upper_limit);

  static void determineClosestConfigs(const RLLKinJoints& joint_angles, RLLKinGlobalConfigs* configs,
                                      const RLLInvKinOptions& options);
};

#endif  // RLL_KINEMATICS_REDUNDANCY_RESOLUTION_H
