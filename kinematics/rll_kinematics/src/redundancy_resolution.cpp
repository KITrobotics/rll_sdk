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

RLLKinMsg RLLRedundancyResolution::ik(const RLLKinJoints& seed_state, RLLKinPoseConfig* ik_pose,
                                      RLLKinSolutions* solutions, const RLLInvKinOptions& options) const
{
  solutions->clear();

  if (!initialized())
  {
    return RLLKinMsg::NOT_INITIALIZED;
  }

  if (!ik_pose->pose.allFinite() || !std::isfinite(ik_pose->arm_angle) || !ik_pose->config.valid() ||
      !seed_state.allFinite())
  {
    return RLLKinMsg::INVALID_INPUT;
  }

  switch (options.method)
  {
    case RLLInvKinOptions::POSITION_RESOLUTION_EXP:
      switch (options.global_configuration_mode)
      {
        case RLLInvKinOptions::KEEP_CURRENT_GLOBAL_CONFIG:
          return ikExpFixedConfig(seed_state, ik_pose, solutions, options);
        default:
          return ikClosestConfig(seed_state, ik_pose, solutions, options,
                                 &RLLRedundancyResolution::optimizationPositionExp);
      }
    case RLLInvKinOptions::ARM_ANGLE_FIXED:
      switch (options.global_configuration_mode)
      {
        case RLLInvKinOptions::KEEP_CURRENT_GLOBAL_CONFIG:
          return ikFixedArmAngleFixedConfig(seed_state, ik_pose, solutions);
        default:
          return ikFixedArmAngle(seed_state, ik_pose, solutions, options);
      }
  }

  return RLLKinMsg::INVALID_INPUT;
}

RLLKinMsg RLLRedundancyResolution::ikFixedArmAngle(const RLLKinJoints& seed_state, RLLKinPoseConfig* ik_pose,
                                                   RLLKinSolutions* solutions, const RLLInvKinOptions& options) const
{
  ik_pose->arm_angle = mapAngleInPiRange(ik_pose->arm_angle);

  return ikClosestConfig(seed_state, ik_pose, solutions, options, &RLLRedundancyResolution::optimizationFixedArmAngle);
}

RLLKinMsg RLLRedundancyResolution::ikExpFixedConfig(const RLLKinJoints& seed_state, RLLKinPoseConfig* ik_pose,
                                                    RLLKinSolutions* solutions, const RLLInvKinOptions& options) const
{
  RLLKinJoints solution;

  ik_pose->config.set(seed_state);

  double seed_arm_angle;
  RLLKinMsg result = armAngle(seed_state, ik_pose->config, &seed_arm_angle);
  if (result.error())
  {
    return result;
  }

  RLLInvKinCoeffs coeffs;
  result = setCoeffsWithInitCheck(*ik_pose, &coeffs, &solution[3]);
  if (result.error())
  {
    return result;
  }

  result = optimizationPositionExp(coeffs, options, seed_arm_angle, &ik_pose->arm_angle, &solution);
  solutions->push_back(solution);

  return result;
}

double RLLRedundancyResolution::expResolution(const RLLInvKinOptions& options, const double arm_angle_old,
                                              const double lower_limit, const double upper_limit)
{
  double k = options.position_exp_k;
  double alpha = options.position_exp_alpha;

  return arm_angle_old + k * (upper_limit - lower_limit) / 2.0 *
                             (exp(-alpha * (arm_angle_old - lower_limit) / (upper_limit - lower_limit)) -
                              exp(-alpha * (upper_limit - arm_angle_old) / (upper_limit - lower_limit)));
}

RLLKinMsg RLLRedundancyResolution::optimizationPositionExp(const RLLInvKinCoeffs& coeffs,
                                                           const RLLInvKinOptions& options, double arm_angle_seed,
                                                           double* arm_angle_new, RLLKinJoints* solution) const
{
  RLLKinArmAngleInterval current_interval;

  RLLInvKinNsIntervals feasible_intervals(coeffs);
  computeFeasibleIntervals(&feasible_intervals);

  RLLKinMsg result = feasible_intervals.intervalForArmAngle(&arm_angle_seed, &current_interval);
  if (result.error())
  {
    // No feasible arm angle could be found. The robot could be in a singular position for the goal pose and the arm
    // angle is not defined. Still try to return feasible joint angles by setting the arm angle to zero.
    *arm_angle_new = 0.0;

    return jointAnglesFromFixedArmAngle(*arm_angle_new, coeffs, solution);
  }

  if (kIsEqual(current_interval.lowerLimit(), -M_PI) && kIsEqual(current_interval.upperLimit(), M_PI) &&
      current_interval.overlapping())
  {
    // all arm angles possible, no limits -> keep current arm angle
    *arm_angle_new = arm_angle_seed;

    return jointAnglesFromArmAngle(*arm_angle_new, coeffs, solution, true);
  }

  if (kIsEqual(current_interval.lowerLimit(), current_interval.upperLimit()))
  {
    *arm_angle_new = current_interval.lowerLimit();

    return jointAnglesFromArmAngle(*arm_angle_new, coeffs, solution, true);
  }

  *arm_angle_new = expResolution(options, arm_angle_seed, current_interval.lowerLimit(), current_interval.upperLimit());
  *arm_angle_new = mapAngleInPiRange(*arm_angle_new);

  return jointAnglesFromArmAngle(*arm_angle_new, coeffs, solution, true);
}

RLLKinMsg RLLRedundancyResolution::optimizationFixedArmAngle(const RLLInvKinCoeffs& coeffs,
                                                             const RLLInvKinOptions& /*unused*/, double /*unused*/,
                                                             // NOLINTNEXTLINE readability-non-const-parameter
                                                             double* arm_angle_new, RLLKinJoints* solution) const
{
  // arm angle is fixed, so no optimization needed

  return jointAnglesFromFixedArmAngle(*arm_angle_new, coeffs, solution);
}

RLLKinMsg RLLRedundancyResolution::ikClosestConfig(
    const RLLKinJoints& seed_state, RLLKinPoseConfig* ik_pose, RLLKinSolutions* solutions,
    const RLLInvKinOptions& options,
    RLLKinMsg (RLLRedundancyResolution::*optimize)(const RLLInvKinCoeffs&, const RLLInvKinOptions& options,
                                                   const double, double*, RLLKinJoints*) const) const
{
  RLLKinMsg result;
  // helper matrices for elbow angle >= 0 and < 0
  std::array<RLLInvKinCoeffs, 2> coeffs;
  std::array<double, 2> joint_angle_4;

  RLLKinGlobalConfigs configs;
  determineClosestConfigs(seed_state, &configs, options);

  double seed_arm_angle;
  result = armAngle(seed_state, configs.front(), &seed_arm_angle);
  if (result.error())
  {
    return result;
  }

  double dist_from_seed = std::numeric_limits<double>::infinity();
  boost::container::static_vector<ClosestConfigsSolution, RLL_NUM_GLOBAL_CONFIGS> solutions_data;

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
    RLLKinJoints solution;
    solution[3] = joint_angle_4[index];
    result = (this->*optimize)(coeffs[index], options, arm_angle_start, &ik_pose->arm_angle, &solution);
    if (result.success())
    {
      dist_from_seed = 0.0;
      for (size_t j = 0; j < RLL_NUM_JOINTS; ++j)
      {
        dist_from_seed += fabs(solution[j] - seed_state(j));
      }

      solutions_data.emplace_back(solution, dist_from_seed, *ik_pose, result);
    }

    addRemainingConfigs(i, dist_from_seed, &configs);
  }

  std::sort(solutions_data.begin(), solutions_data.end());

  if (!solutions_data.empty())
  {
    std::transform(solutions_data.begin(), solutions_data.end(), std::back_inserter(*solutions),
                   [](ClosestConfigsSolution const& ccs) { return ccs.solution(); });
    *ik_pose = solutions_data.front().pose();
    return solutions_data.front().result();
  }

  return result;
}

void RLLRedundancyResolution::determineClosestConfigs(const RLLKinJoints& joint_angles, RLLKinGlobalConfigs* configs,
                                                      const RLLInvKinOptions& options)
{
  RLLKinGlobalConfigs& configs_ref = *configs;

  assert(configs_ref.empty());

  if (options.global_configuration_mode == RLLInvKinOptions::RETURN_ALL_GLOBAL_CONFIGS)
  {
    // insert all possible configs
    for (uint8_t i = 0; i < RLL_NUM_GLOBAL_CONFIGS; ++i)
    {
      configs_ref.emplace_back(i);
    }

    return;
  }

  configs_ref.emplace_back(joint_angles);  // keep in current config (preferred)

  // only check for different configurations if respective seed joint angle is close to zero

  if (fabs(joint_angles(1)) < GLOBAL_CONFIG_DISTANCE_TOL)
  {
    configs_ref.emplace_back(configs_ref.front().val() ^ (1 << 0));  // toggle first bit
  }

  if (fabs(joint_angles(3)) < GLOBAL_CONFIG_DISTANCE_TOL)
  {
    size_t current_size = configs_ref.size();
    for (size_t i = 0; i < current_size; ++i)
    {
      configs_ref.emplace_back(configs_ref[i].val() ^ (1 << 1));  // toggle second bit
    }
  }

  if (fabs(joint_angles(5)) < GLOBAL_CONFIG_DISTANCE_TOL)
  {
    size_t current_size = configs_ref.size();
    for (size_t i = 0; i < current_size; ++i)
    {
      configs_ref.emplace_back(configs_ref[i].val() ^ (1 << 2));  // toggle third bit
    }
  }
}
