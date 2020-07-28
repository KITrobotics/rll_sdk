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
    POSITION_RESOLUTION_EXP,     // using the method from Faria et al. based on exponential function
    RESOLUTION_MULTI_OBJECTIVE,  // multi-objective redundancy resolution with minimization of joint velocities and
                                 // accelerations
    ARM_ANGLE_FIXED              // desired arm angle is given
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

  Method method = RESOLUTION_MULTI_OBJECTIVE;
  GlobalConfigurationControl global_configuration_mode = SELECT_NEAREST_GLOBAL_CONFIG;

  // parameters of the position-based resolution
  double position_exp_k = 0.016;
  double position_exp_alpha = 4.3;

  // parameters for the multi-objective optimization
  double delta_t_desired = 0.04;  // desired time step; if set to zero or negative, the minimal admissible value
                                  // according to velocity/acceleration limits will be used
  double joint_velocity_scaling_factor = 1.0;
  double joint_acceleration_scaling_factor = 1.0;
  bool minimize_acceleration = true;
  bool use_numerical_solver = false;
  double d_v = 1.0;
  double d_a = 1.0;
  double d_l = 1.0;
};

class RLLRedundancyResolution : public RLLInverseKinematics
{
public:
  RLLRedundancyResolution() = default;

  RLLKinMsg ik(const RLLKinSeedState& seed_state, RLLKinPoseConfig* ik_pose, RLLKinSolutions* solution,
               const RLLInvKinOptions& options) const;

protected:
  // redundancy resolution using fixed arm angle and variable global config
  RLLKinMsg ikFixedArmAngle(const RLLKinSeedState& seed_state, RLLKinPoseConfig* ik_pose, RLLKinSolutions* solution,
                            const RLLInvKinOptions& options) const;
  // redundancy resolution with fixed global config
  RLLKinMsg ikFixedConfig(const RLLKinSeedState& seed_state, RLLKinPoseConfig* ik_pose, RLLKinSolutions* solution,
                          const RLLInvKinOptions& options) const;

  // get closest solution to seed-state using optimization of arm angle defined in optimize()
  RLLKinMsg ikClosestConfig(const RLLKinSeedState& seed_state, RLLKinPoseConfig* ik_pose, RLLKinSolutions* solution,
                            const RLLInvKinOptions& options) const;
  // actual redundancy resolution
  RLLKinMsg redundancyResolution(const RLLInvKinCoeffs& coeffs, const RLLInvKinOptions& options, double arm_angle_seed,
                                 double* arm_angle_new, RLLKinJoints* solution) const;
  // redundancy resolution using multi-objective optimization
  RLLKinMsg optimizationMultiObjective(const RLLInvKinCoeffs& coeffs, const RLLInvKinOptions& options,
                                       double arm_angle_seed, double* arm_angle_new, RLLKinJoints* solution) const;
  // redundancy resolution using exponential function
  RLLKinMsg optimizationPositionExp(const RLLInvKinCoeffs& coeffs, const RLLInvKinOptions& options,
                                    double arm_angle_seed, double* arm_angle_new, RLLKinJoints* solution) const;
  RLLKinMsg optimizationFixedArmAngle(const RLLInvKinCoeffs& coeffs, const RLLInvKinOptions& options,
                                      double arm_angle_seed, double* arm_angle_new, RLLKinJoints* solution) const;

  double multiObjectiveResolution(const RLLInvKinCoeffs& coeffs, const RLLInvKinOptions& options, double arm_angle_old,
                                  const RLLKinArmAngleInterval& current_interval) const;
  static double expResolution(const RLLInvKinOptions& options, double arm_angle_old, double lower_limit,
                              double upper_limit);

  static void determineClosestConfigs(const RLLKinJoints& joint_angles, RLLKinGlobalConfigs* configs,
                                      const RLLInvKinOptions& options);

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
};

class RLLKinMultiObjOptimization : public RLLKinematicsBase
{
public:
  RLLKinMultiObjOptimization(RLLInvKinCoeffs coeffs, const RLLInvKinOptions& options, const double arm_angle_old,
                             const RLLKinArmAngleInterval& arm_angle_interval)
    : coeffs_(std::move(coeffs))
    , options_(options)
    , arm_angle_old_(arm_angle_old)
    , arm_angle_interval_(arm_angle_interval)
  {
  }

  double optimalArmAngle(const RLLKinJoints& joint_velocity_limits, const RLLKinJoints& joint_acceleration_limits);

protected:
  double optimalArmAngleClosedForm() const;
  double optimalArmAngleNumerical() const;
  void limitWeightsArmAngle(double* w_dis, double* w_tilde_l, double* arm_angle_l) const;
  void dynamicWeights(double w_dis, double* w_v, double* w_a) const;
  void closedFormArmAngleVelAccel(double* arm_angle_v_n, double* arm_angle_v_d, double* arm_angle_a_n,
                                  double* arm_angle_a_d) const;
  double costFunction(double arm_angle) const;
  void deltaT(double arm_angle_v, double arm_angle_a, double* delta_t_v, double* delta_t_a) const;

  double maxVelocity(size_t index) const
  {
    return options_.joint_velocity_scaling_factor * joint_velocity_limits_(index);
  }

  double maxAcceleration(size_t index) const
  {
    return options_.joint_acceleration_scaling_factor * joint_acceleration_limits_(index);
  }

  class CostFunctionNumericalSolverFunctor
  {
  public:
    explicit CostFunctionNumericalSolverFunctor(const RLLKinMultiObjOptimization& optimization)
      : optimization_(optimization)
    {
    }

    double operator()(double const& arm_angle)
    {
      return optimization_.costFunction(arm_angle);
    }

  private:
    const RLLKinMultiObjOptimization& optimization_;
  };

private:
  RLLInvKinCoeffs coeffs_;
  RLLInvKinOptions options_;
  double arm_angle_old_;
  RLLKinArmAngleInterval arm_angle_interval_;

  RLLKinJoints joint_velocity_limits_;
  RLLKinJoints joint_acceleration_limits_;
};

#endif  // RLL_KINEMATICS_REDUNDANCY_RESOLUTION_H
