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

#include <rll_kinematics/redundancy_resolution.h>

RLLKinMsg RLLRedundancyResolution::ik(const RLLKinJoints& seed_state, RLLKinPoseConfig* ik_pose, RLLKinJoints* solution,
                                      const RLLInvKinOptions& options) const
{
  switch (options.method)
  {
    case RLLInvKinOptions::POSITION_RESOLUTION_EXP:
      if (options.keep_global_configuration)
      {
        return ikExpFixedConfig(seed_state, ik_pose, solution, options);
      }

      return ikClosestConfig(seed_state, ik_pose, solution, options, &RLLRedundancyResolution::optimizationPositionExp);
    case RLLInvKinOptions::ARM_ANGLE_FIXED:
      if (options.keep_global_configuration)
      {
        return ikFixedArmAngleFixedConfig(ik_pose, solution);
      }

      return ikFixedArmAngle(seed_state, ik_pose, solution, options);
  }

  return RLLKinMsg::INVALID_INPUT;
}

RLLKinMsg RLLRedundancyResolution::ikFixedArmAngle(const RLLKinJoints& seed_state, RLLKinPoseConfig* ik_pose,
                                                   RLLKinJoints* solution, const RLLInvKinOptions& options) const
{
  ik_pose->arm_angle = mapAngleInPiRange(ik_pose->arm_angle);

  return ikClosestConfig(seed_state, ik_pose, solution, options, &RLLRedundancyResolution::optimizationFixedArmAngle);
}

RLLKinMsg RLLRedundancyResolution::ikExpFixedConfig(const RLLKinJoints& seed_state, RLLKinPoseConfig* ik_pose,
                                                    RLLKinJoints* solution, const RLLInvKinOptions& options) const
{
  RLLKinJoints& solution_ref = *solution;

  ik_pose->config.set(seed_state);

  double seed_arm_angle;
  RLLKinMsg result = armAngle(seed_state, ik_pose->config, &seed_arm_angle);
  if (result.error())
  {
    return result;
  }

  RLLInvKinCoeffs coeffs;
  result = setCoeffsWithInitCheck(*ik_pose, &coeffs, &solution_ref[3]);
  if (result.error())
  {
    return result;
  }

  return optimizationPositionExp(coeffs, options, seed_arm_angle, &ik_pose->arm_angle, solution);
}

double RLLRedundancyResolution::expResolution(const RLLInvKinOptions& options, const double arm_angle_old,
                                              const double lower_limit, const double upper_limit) const
{
  double k = options.position_exp_k;
  double alpha = options.position_exp_alpha;

  return arm_angle_old + k * (upper_limit - lower_limit) / 2.0 *
                             (exp(-alpha * (arm_angle_old - lower_limit) / (upper_limit - lower_limit)) -
                              exp(-alpha * (upper_limit - arm_angle_old) / (upper_limit - lower_limit)));
}

RLLKinMsg RLLRedundancyResolution::optimizationPositionExp(const RLLInvKinCoeffs& coeffs,
                                                           const RLLInvKinOptions& options, const double arm_angle_seed,
                                                           double* arm_angle_new, RLLKinJoints* solution) const
{
  RLLKinArmAngleInterval current_interval;
  double arm_angle_old = arm_angle_seed;

  RLLInvKinNsIntervals feasible_intervals(coeffs);
  computeFeasibleIntervals(&feasible_intervals);

  RLLKinMsg result = feasible_intervals.intervalForArmAngle(&arm_angle_old, &current_interval);
  if (result.error())
  {
    return result;
  }

  if (current_interval.lower_limit == -M_PI && current_interval.upper_limit == M_PI && current_interval.overlap)
  {
    // all arm angles possible, no limits -> keep current arm angle
    *arm_angle_new = arm_angle_old;

    return RLLKinMsg::SUCCESS;
  }

  if (current_interval.lower_limit == current_interval.upper_limit)
  {
    *arm_angle_new = current_interval.lower_limit;

    return RLLKinMsg::SUCCESS;
  }

  *arm_angle_new = expResolution(options, arm_angle_old, current_interval.lower_limit, current_interval.upper_limit);
  *arm_angle_new = mapAngleInPiRange(*arm_angle_new);

  return jointAnglesFromArmAngle(*arm_angle_new, coeffs, solution, true);
}

RLLKinMsg RLLRedundancyResolution::optimizationFixedArmAngle(const RLLInvKinCoeffs& coeffs,
                                                             const RLLInvKinOptions& /*unused*/, double /*unused*/,
                                                             // NOLINTNEXTLINE readability-non-const-parameter
                                                             double* arm_angle_new, RLLKinJoints* solution) const
{
  // arm angle is fixed, so no optimization needed

  return jointAnglesFromArmAngle(*arm_angle_new, coeffs, solution);
}

RLLKinMsg RLLRedundancyResolution::ikClosestConfig(
    const RLLKinJoints& seed_state, RLLKinPoseConfig* ik_pose, RLLKinJoints* solution, const RLLInvKinOptions& options,
    RLLKinMsg (RLLRedundancyResolution::*optimize)(const RLLInvKinCoeffs&, const RLLInvKinOptions& options,
                                                   const double, double*, RLLKinJoints*) const) const
{
  RLLKinMsg result_last;
  RLLKinMsg result;
  // helper matrices for elbow angle >= 0 and < 0
  std::array<RLLInvKinCoeffs, 2> coeffs;
  std::array<double, 2> joint_angle_4;

  RLLKinGlobalConfigs configs;
  determineClosestConfigs(seed_state, &configs);

  double seed_arm_angle;
  result = armAngle(seed_state, configs.front(), &seed_arm_angle);
  if (result.error())
  {
    return result;
  }

  double dist_from_seed_last = std::numeric_limits<double>::infinity();
  double dist_from_seed = std::numeric_limits<double>::infinity();
  RLLKinJoints solution_last;
  RLLKinJoints& solution_ref = *solution;

  for (size_t i = 0; i < configs.size(); ++i)
  {
    int index = configs[i].indexGC4();
    ik_pose->config.set(configs[i].val());

    result = setCoeffsWithInitCheck(*ik_pose, &coeffs[index], &joint_angle_4[index]);
    if (result.error())
    {
      continue;
    }

    double arm_angle_start = mapArmAngleForGC4(configs[0], configs[i], seed_arm_angle);
    ik_pose->arm_angle = mapArmAngleForGC4(configs[0], configs[i], ik_pose->arm_angle);
    solution_ref[3] = joint_angle_4[index];
    result = (this->*optimize)(coeffs[index], options, arm_angle_start, &ik_pose->arm_angle, solution);
    if (result.error())
    {
      dist_from_seed = std::numeric_limits<double>::infinity();
    }
    else
    {
      dist_from_seed = 0.0;
      for (size_t j = 0; j < RLL_NUM_JOINTS; ++j)
      {
        dist_from_seed += fabs(solution_ref[j] - seed_state(j));
      }
    }

    if (dist_from_seed_last > dist_from_seed)
    {
      // choose solution with lowest distance from seed
      dist_from_seed_last = dist_from_seed;
      solution_last = solution_ref;
      result_last = result;
    }

    addRemainingConfigs(i, dist_from_seed_last, &configs);
  }

  if (dist_from_seed_last < std::numeric_limits<double>::infinity())
  {
    solution_ref = solution_last;
    return result_last;
  }

  return result;
}
