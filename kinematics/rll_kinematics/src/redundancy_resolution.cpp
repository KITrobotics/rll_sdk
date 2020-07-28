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

#include <boost/math/tools/minima.hpp>  // one dimensional minimization

#include <rll_kinematics/redundancy_resolution.h>

RLLKinMsg RLLRedundancyResolution::ik(const RLLKinSeedState& seed_state, RLLKinPoseConfig* ik_pose,
                                      RLLKinSolutions* solutions, const RLLInvKinOptions& options) const
{
  solutions->clear();

  if (!initialized())
  {
    return RLLKinMsg::NOT_INITIALIZED;
  }

  if (!ik_pose->pose.allFinite() || !std::isfinite(ik_pose->arm_angle) || !ik_pose->config.valid())
  {
    return RLLKinMsg::INVALID_INPUT;
  }

  if (seed_state.empty())
  {
    return RLLKinMsg::INVALID_INPUT;
  }

  for (size_t i = 0; i < seed_state.size(); ++i)
  {
    if (!seed_state[i].allFinite())
    {
      return RLLKinMsg::INVALID_INPUT;
    }
  }

  switch (options.method)
  {
    case RLLInvKinOptions::ARM_ANGLE_FIXED:
      switch (options.global_configuration_mode)
      {
        case RLLInvKinOptions::KEEP_CURRENT_GLOBAL_CONFIG:
          return ikFixedArmAngleFixedConfig(seed_state.front(), ik_pose, solutions);
        default:
          return ikFixedArmAngle(seed_state, ik_pose, solutions, options);
      }
    default:
      switch (options.global_configuration_mode)
      {
        case RLLInvKinOptions::KEEP_CURRENT_GLOBAL_CONFIG:
          return ikFixedConfig(seed_state, ik_pose, solutions, options);
        default:
          return ikClosestConfig(seed_state, ik_pose, solutions, options);
      }
  }

  return RLLKinMsg::INVALID_INPUT;
}

RLLKinMsg RLLRedundancyResolution::ikFixedArmAngle(const RLLKinSeedState& seed_state, RLLKinPoseConfig* ik_pose,
                                                   RLLKinSolutions* solutions, const RLLInvKinOptions& options) const
{
  ik_pose->arm_angle = mapAngleInPiRange(ik_pose->arm_angle);

  return ikClosestConfig(seed_state, ik_pose, solutions, options);
}

RLLKinMsg RLLRedundancyResolution::ikFixedConfig(const RLLKinSeedState& seed_state, RLLKinPoseConfig* ik_pose,
                                                 RLLKinSolutions* solutions, const RLLInvKinOptions& options) const
{
  RLLKinJoints solution;

  ik_pose->config.set(seed_state.front());

  double seed_arm_angle;
  RLLKinMsg result = armAngle(seed_state.front(), ik_pose->config, &seed_arm_angle);
  if (result.error())
  {
    return result;
  }

  RLLInvKinCoeffs coeffs(seed_state);
  result = setCoeffsWithInitCheck(*ik_pose, &coeffs, &solution[3]);
  if (result.error())
  {
    return result;
  }

  result = redundancyResolution(coeffs, options, seed_arm_angle, &ik_pose->arm_angle, &solution);
  solutions->push_back(solution);

  return result;
}

RLLKinMsg RLLRedundancyResolution::redundancyResolution(const RLLInvKinCoeffs& coeffs, const RLLInvKinOptions& options,
                                                        double arm_angle_seed, double* arm_angle_new,
                                                        RLLKinJoints* solution) const
{
  switch (options.method)
  {
    case RLLInvKinOptions::RESOLUTION_MULTI_OBJECTIVE:
      if (coeffs.seedState().size() < RLL_MAX_NUM_SEED_TIME_STEPS)
      {
        return RLLKinMsg::INVALID_INPUT;
      }

      return optimizationMultiObjective(coeffs, options, arm_angle_seed, arm_angle_new, solution);
    case RLLInvKinOptions::POSITION_RESOLUTION_EXP:
      return optimizationPositionExp(coeffs, options, arm_angle_seed, arm_angle_new, solution);
    case RLLInvKinOptions::ARM_ANGLE_FIXED:
      return optimizationFixedArmAngle(coeffs, options, arm_angle_seed, arm_angle_new, solution);
  }

  return RLLKinMsg::INVALID_INPUT;
}

double RLLRedundancyResolution::multiObjectiveResolution(const RLLInvKinCoeffs& coeffs, const RLLInvKinOptions& options,
                                                         const double arm_angle_old,
                                                         const RLLKinArmAngleInterval& current_interval) const
{
  RLLKinMultiObjOptimization optimization(coeffs, options, arm_angle_old, current_interval);

  return optimization.optimalArmAngle(jointVelocityLimits(), jointAccelerationLimits());
}

RLLKinMsg RLLRedundancyResolution::optimizationMultiObjective(const RLLInvKinCoeffs& coeffs,
                                                              const RLLInvKinOptions& options, double arm_angle_seed,
                                                              double* arm_angle_new, RLLKinJoints* solution) const
{
  RLLKinArmAngleInterval current_interval;
  double fallback_arm_angle;

  RLLInvKinNsIntervals feasible_intervals(coeffs);
  computeFeasibleIntervals(&feasible_intervals);

  RLLKinMsg result = feasible_intervals.intervalForArmAngle(&arm_angle_seed, &current_interval, &fallback_arm_angle);
  if (result.error() || result.val() == RLLKinMsg::ARMANGLE_NOT_IN_SAME_INTERVAL)
  {
    *arm_angle_new = fallback_arm_angle;
    return jointAnglesFromFixedArmAngle(fallback_arm_angle, coeffs, solution);
  }

  if (!dynamicLimitsSet())
  {
    return RLLKinMsg::INVALID_INPUT;
  }

  if (options.use_numerical_solver && options.delta_t_desired <= 0.0)
  {
    // delta_t_desired needs to be set for the numerical solver
    return RLLKinMsg::INVALID_INPUT;
  }

  *arm_angle_new = multiObjectiveResolution(coeffs, options, arm_angle_seed, current_interval);

  *arm_angle_new = mapAngleInPiRange(*arm_angle_new);

  return jointAnglesFromArmAngle(*arm_angle_new, coeffs, solution, true);
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
  double fallback_arm_angle;

  RLLInvKinNsIntervals feasible_intervals(coeffs);
  computeFeasibleIntervals(&feasible_intervals);

  RLLKinMsg result = feasible_intervals.intervalForArmAngle(&arm_angle_seed, &current_interval, &fallback_arm_angle);
  if (result.error() || result.val() == RLLKinMsg::ARMANGLE_NOT_IN_SAME_INTERVAL)
  {
    *arm_angle_new = fallback_arm_angle;
    return jointAnglesFromFixedArmAngle(fallback_arm_angle, coeffs, solution);
  }

  if (kIsEqual(current_interval.lowerLimit(), -M_PI) && kIsEqual(current_interval.upperLimit(), M_PI) &&
      current_interval.overlapping())
  {
    // all arm angles possible, no limits -> keep current arm angle
    *arm_angle_new = arm_angle_seed;

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

RLLKinMsg RLLRedundancyResolution::ikClosestConfig(const RLLKinSeedState& seed_state, RLLKinPoseConfig* ik_pose,
                                                   RLLKinSolutions* solutions, const RLLInvKinOptions& options) const
{
  RLLKinMsg result;
  // helper matrices for elbow angle >= 0 and < 0
  std::array<RLLInvKinCoeffs, 2> coeffs;
  coeffs.front().setSeedState(seed_state);
  coeffs.back().setSeedState(seed_state);
  std::array<double, 2> joint_angle_4;

  RLLKinGlobalConfigs configs;
  determineClosestConfigs(seed_state.front(), &configs, options);

  double seed_arm_angle;
  result = armAngle(seed_state.front(), configs.front(), &seed_arm_angle);
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

    RLLKinJoints solution;
    solution[3] = joint_angle_4[index];
    result = redundancyResolution(coeffs[index], options, arm_angle_start, &ik_pose->arm_angle, &solution);
    if (result.success())
    {
      dist_from_seed = 0.0;
      for (size_t j = 0; j < RLL_NUM_JOINTS; ++j)
      {
        dist_from_seed += fabs(solution[j] - seed_state.front()(j));
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

double RLLKinMultiObjOptimization::optimalArmAngle(const RLLKinJoints& joint_velocity_limits,
                                                   const RLLKinJoints& joint_acceleration_limits)
{
  double arm_angle;
  joint_velocity_limits_ = joint_velocity_limits;
  joint_acceleration_limits_ = joint_acceleration_limits;

  if (options_.use_numerical_solver)
  {
    arm_angle = optimalArmAngleNumerical();
  }
  else
  {
    arm_angle = optimalArmAngleClosedForm();
  }

  if (arm_angle <= arm_angle_interval_.lowerLimit() || arm_angle >= arm_angle_interval_.upperLimit())
  {
    // seed state and goal are too far apart and optimization did not find a feasible arm angle, set to middle between
    // intervals
    arm_angle = (arm_angle_interval_.upperLimit() + arm_angle_interval_.lowerLimit()) / 2.0;
  }

  return arm_angle;
}

double RLLKinMultiObjOptimization::optimalArmAngleClosedForm() const
{
  double w_dis, w_tilde_l, arm_angle_l;
  limitWeightsArmAngle(&w_dis, &w_tilde_l, &arm_angle_l);

  double arm_angle_v_n, arm_angle_v_d, arm_angle_a_n, arm_angle_a_d;
  closedFormArmAngleVelAccel(&arm_angle_v_n, &arm_angle_v_d, &arm_angle_a_n, &arm_angle_a_d);

  double arm_angle_v = arm_angle_v_n / arm_angle_v_d;
  double arm_angle_a = arm_angle_a_n / arm_angle_a_d;
  double delta_t_v, delta_t_a;
  deltaT(arm_angle_v, arm_angle_a, &delta_t_v, &delta_t_a);

  double w_v, w_a;
  dynamicWeights(w_dis, &w_v, &w_a);
  double w_tilde_v = w_v / pow(delta_t_v, 2);
  double w_tilde_a = w_a / pow(delta_t_a, 4);

  return (w_tilde_v * arm_angle_v_n + w_tilde_a * arm_angle_a_n + w_tilde_l * arm_angle_l) /
         (w_tilde_v * arm_angle_v_d + w_tilde_a * arm_angle_a_d + w_tilde_l);
}

double RLLKinMultiObjOptimization::optimalArmAngleNumerical() const
{
  CostFunctionNumericalSolverFunctor functor(*this);

  const boost::uintmax_t MAX_IT = 1000 * 1000;
  boost::uintmax_t it = MAX_IT;
  const int DOUBLE_BITS = std::numeric_limits<double>::digits;
  std::pair<double, double> result = boost::math::tools::brent_find_minima(
      functor, arm_angle_interval_.lowerLimit(), arm_angle_interval_.upperLimit(), DOUBLE_BITS, it);

  return result.first;
}

void RLLKinMultiObjOptimization::limitWeightsArmAngle(double* w_dis, double* w_tilde_l, double* arm_angle_l) const
{
  if (kIsEqual(arm_angle_interval_.lowerLimit(), -M_PI) && kIsEqual(arm_angle_interval_.upperLimit(), M_PI) &&
      arm_angle_interval_.overlapping())
  {
    // all arm angles possible, no limits -> ignore limits objective
    *w_dis = 0.0;
    *w_tilde_l = 0.0;
    *arm_angle_l = arm_angle_old_;

    return;
  }

  double upper_limit = arm_angle_interval_.upperLimit();
  double lower_limit = arm_angle_interval_.lowerLimit();

  *w_dis = (fabs(arm_angle_old_ - (upper_limit + lower_limit) / 2)) / ((upper_limit - lower_limit) / 2);
  *w_tilde_l = 4 / pow(upper_limit - lower_limit, 2) * pow(*w_dis, options_.d_l);
  *arm_angle_l = (upper_limit + lower_limit) / 2;
}

void RLLKinMultiObjOptimization::closedFormArmAngleVelAccel(double* arm_angle_v_n, double* arm_angle_v_d,
                                                            double* arm_angle_a_n, double* arm_angle_a_d) const
{
  std::array<double, RLL_NUM_JOINTS> a_v;
  for (size_t i = 0; i < RLL_NUM_JOINTS; ++i)
  {
    a_v[i] = 1 / (7 * pow(maxVelocity(i), 2));
  }

  std::array<double, RLL_NUM_JOINTS> deriv_theta_psi, delta_theta_p;
  for (uint8_t i = 0; i < RLL_NUM_JOINTS_P; ++i)
  {
    deriv_theta_psi[i * 2] = coeffs_.jointDerivativePivot(i, arm_angle_old_);
    double theta_p_psi_old = coeffs_.jointAnglePivot(i, arm_angle_old_);
    delta_theta_p[i * 2] = theta_p_psi_old - coeffs_.seedState().front()(i * 2);
  }
  for (uint8_t i = 0; i < RLL_NUM_JOINTS_H; ++i)
  {
    double theta_p_psi_old = coeffs_.jointAngleHinge(i, arm_angle_old_);
    deriv_theta_psi[i * 4 + 1] = coeffs_.jointDerivativeHinge(i, arm_angle_old_, theta_p_psi_old);
    delta_theta_p[i * 4 + 1] = theta_p_psi_old - coeffs_.seedState().front()(i * 4 + 1);
  }
  // no change for elbow joint angle in variation of arm angle
  deriv_theta_psi[3] = 0.0;
  delta_theta_p[3] = 0.0;

  *arm_angle_v_n = 0.0;
  for (size_t i = 0; i < RLL_NUM_JOINTS; ++i)
  {
    *arm_angle_v_n += a_v[i] * deriv_theta_psi[i] * (delta_theta_p[i] - deriv_theta_psi[i] * arm_angle_old_);
  }
  *arm_angle_v_n = -*arm_angle_v_n;

  *arm_angle_v_d = 0.0;
  for (size_t i = 0; i < RLL_NUM_JOINTS; ++i)
  {
    *arm_angle_v_d += a_v[i] * pow(deriv_theta_psi[i], 2);
  }

  if (!options_.minimize_acceleration)
  {
    *arm_angle_a_n = 0.0;
    *arm_angle_a_d = 0.0;

    return;
  }

  std::array<double, RLL_NUM_JOINTS> a_a;
  for (size_t i = 0; i < RLL_NUM_JOINTS; ++i)
  {
    a_a[i] = 1 / (7 * pow(maxAcceleration(i), 2));
  }

  *arm_angle_a_n = 0.0;
  for (size_t i = 0; i < RLL_NUM_JOINTS; ++i)
  {
    *arm_angle_a_n += a_a[i] * deriv_theta_psi[i] *
                      (delta_theta_p[i] - deriv_theta_psi[i] * arm_angle_old_ -
                       (coeffs_.seedState().front()(i) - coeffs_.seedState()[1](i)));
  }
  *arm_angle_a_n = -*arm_angle_a_n;

  *arm_angle_a_d = 0.0;
  for (size_t i = 0; i < RLL_NUM_JOINTS; ++i)
  {
    *arm_angle_a_d += a_a[i] * pow(deriv_theta_psi[i], 2);
  }
}

void RLLKinMultiObjOptimization::dynamicWeights(double w_dis, double* w_v, double* w_a) const
{
  *w_v = pow(1 - w_dis, options_.d_v);

  if (options_.minimize_acceleration)
  {
    *w_a = pow(1 - w_dis, options_.d_a);
  }
  else
  {
    *w_a = 0.0;
  }
}

double RLLKinMultiObjOptimization::costFunction(double arm_angle) const
{
  double w_dis, w_tilde_l, arm_angle_l;
  limitWeightsArmAngle(&w_dis, &w_tilde_l, &arm_angle_l);
  double w_l = w_dis;

  double w_v, w_a;
  dynamicWeights(w_dis, &w_v, &w_a);

  double f_l =
      pow((arm_angle - arm_angle_l) / ((arm_angle_interval_.upperLimit() - arm_angle_interval_.lowerLimit()) / 2), 2);

  std::array<double, RLL_NUM_JOINTS> delta_psi;
  for (uint8_t i = 0; i < RLL_NUM_JOINTS_P; ++i)
  {
    double joint_angle = coeffs_.jointAnglePivot(i, arm_angle);
    delta_psi[i * 2] = joint_angle - coeffs_.seedState().front()(i * 2);
  }

  for (uint8_t i = 0; i < RLL_NUM_JOINTS_H; ++i)
  {
    double joint_angle = coeffs_.jointAngleHinge(i, arm_angle);
    delta_psi[i * 4 + 1] = joint_angle - coeffs_.seedState().front()(i * 4 + 1);
  }

  double delta_t = options_.delta_t_desired;

  double f_v = 0.0;
  for (size_t i = 0; i < RLL_NUM_JOINTS; ++i)
  {
    f_v += pow(delta_psi[i] / (maxVelocity(i) * delta_t), 2);
  }
  f_v *= 1.0 / 7.0;

  double f_a = 0.0;
  for (size_t i = 0; i < RLL_NUM_JOINTS; ++i)
  {
    f_a += pow((delta_psi[i] - coeffs_.seedState().front()(i) + coeffs_.seedState()[1](i)) /
                   (maxAcceleration(i) * pow(delta_t, 2)),
               2);
  }
  f_a *= 1.0 / 7.0;

  return w_v * f_v + w_a * f_a + w_l * f_l;
}

void RLLKinMultiObjOptimization::deltaT(const double arm_angle_v, const double arm_angle_a, double* delta_t_v,
                                        double* delta_t_a) const
{
  if (options_.delta_t_desired > 0.0)
  {
    *delta_t_v = *delta_t_a = options_.delta_t_desired;

    return;
  }

  *delta_t_v = 0.0;
  for (uint8_t i = 0; i < RLL_NUM_JOINTS_P; ++i)
  {
    double joint_angle_v = coeffs_.jointAnglePivot(i, arm_angle_v);
    double delta_t = fabs(joint_angle_v - coeffs_.seedState().front()(i * 2)) / maxVelocity(i * 2);
    if (delta_t > *delta_t_v)
    {
      *delta_t_v = delta_t;
    }
  }
  for (uint8_t i = 0; i < RLL_NUM_JOINTS_H; ++i)
  {
    double joint_angle_v = coeffs_.jointAngleHinge(i, arm_angle_v);
    double delta_t = fabs(joint_angle_v - coeffs_.seedState().front()(i * 4 + 1)) / maxVelocity(i * 4 + 1);
    if (delta_t > *delta_t_v)
    {
      *delta_t_v = delta_t;
    }
  }

  *delta_t_a = 0.0;

  if (!options_.minimize_acceleration)
  {
    return;
  }

  // TODO(wolfgang): probably wrong, delta_t not comparable this way
  // e.g. check if a_max * delta_T^2 > v_max * delta_T

  for (uint8_t i = 0; i < RLL_NUM_JOINTS_P; ++i)
  {
    double joint_angle_a = coeffs_.jointAnglePivot(i, arm_angle_a);
    double delta_t = fabs(joint_angle_a - coeffs_.seedState().front()(i * 2)) / maxAcceleration(i * 2);
    if (delta_t > *delta_t_a)
    {
      *delta_t_a = delta_t;
    }
  }
  for (uint8_t i = 0; i < RLL_NUM_JOINTS_H; ++i)
  {
    double joint_angle_a = coeffs_.jointAngleHinge(i, arm_angle_a);
    double delta_t = sqrt(fabs(joint_angle_a - coeffs_.seedState().front()(i * 4 + 1)) / maxAcceleration(i * 4 + 1));
    if (delta_t > *delta_t_a)
    {
      *delta_t_a = delta_t;
    }
  }
}
