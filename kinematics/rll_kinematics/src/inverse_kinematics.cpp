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

#include <rll_kinematics/inverse_kinematics.h>

const double RLLInverseKinematics::GLOBAL_CONFIG_DISTANCE_TOL = 5.0 / 180.0 * M_PI;

RLLKinMsg RLLInverseKinematics::ikFixedArmAngleFixedConfig(const RLLKinJoints& seed_state, RLLKinPoseConfig* eef_pose,
                                                           RLLKinSolutions* solutions) const
{
  RLLInvKinCoeffs coeffs;
  RLLKinJoints solution;

  eef_pose->config.set(seed_state);
  eef_pose->arm_angle = mapAngleInPiRange(eef_pose->arm_angle);

  RLLKinMsg result = setHelperMatrices(*eef_pose, &coeffs, &solution[3]);
  if (result.error())
  {
    return result;
  }

  coeffs.setGC(eef_pose->config);

  result = jointAnglesFromFixedArmAngle(eef_pose->arm_angle, coeffs, &solution);
  solutions->push_back(solution);

  return result;
}

RLLKinMsg RLLInverseKinematics::jointAnglesFromFixedArmAngle(double arm_angle, const RLLInvKinCoeffs& coeffs,
                                                             RLLKinJoints* joint_angles) const
{
  RLLKinMsg result = jointAnglesFromArmAngle(arm_angle, coeffs, joint_angles);
  if (result.error() && kZero((*joint_angles)[3]))
  {
    // global config at joint 4 cannot be uniquely identified, also try mapped arm angle for different GC4
    // This ensures that pivot joints are not flipped by PI
    // force a mapping by using configs with different GC4
    arm_angle = mapArmAngleForGC4(RLLKinGlobalConfig(0), RLLKinGlobalConfig(2), arm_angle);
    return jointAnglesFromArmAngle(arm_angle, coeffs, joint_angles);
  }

  return result;
}

RLLKinMsg RLLInverseKinematics::jointAnglesFromArmAngle(const double arm_angle, const RLLInvKinCoeffs& coeffs,
                                                        RLLKinJoints* joint_angles, const bool assert_limits) const
{
  RLLKinJoints& joint_angles_ref = *joint_angles;

  for (uint8_t i = 0; i < RLL_NUM_JOINTS_P; ++i)
  {
    joint_angles_ref[i * 2] = coeffs.jointAnglePivot(i, arm_angle);
  }

  for (uint8_t i = 0; i < RLL_NUM_JOINTS_H; ++i)
  {
    joint_angles_ref[i * 4 + 1] = coeffs.jointAngleHinge(i, arm_angle);
  }

  if (assert_limits)
  {
    assert(!jointLimitsViolated(joint_angles_ref));
  }
  else if (jointLimitsViolated(joint_angles_ref))
  {
    return RLLKinMsg::JOINT_LIMIT_VIOLATED;
  }

  return checkSingularities(joint_angles_ref, coeffs.sw());
}

RLLKinMsg RLLInverseKinematics::setCoeffsWithInitCheck(const RLLKinPoseConfig& eef_pose, RLLInvKinCoeffs* coeffs,
                                                       double* joint_angle_4, const bool set_derivatives) const
{
  if (coeffs->initError().error())
  {
    return coeffs->initError();
  }

  if (coeffs->initialized())
  {
    *joint_angle_4 = coeffs->jointAngle4();
    // still update the config
    coeffs->setGC(eef_pose.config);

    return RLLKinMsg::SUCCESS;
  }

  RLLKinMsg result = setHelperMatrices(eef_pose, coeffs, joint_angle_4);
  if (result.error())
  {
    coeffs->setInitError(result);
    return result;
  }

  if (set_derivatives)
  {
    coeffs->preparePivotDerivatives();
  }

  coeffs->setGC(eef_pose.config);
  coeffs->setInitialized();

  return RLLKinMsg::SUCCESS;
}

RLLKinMsg RLLInverseKinematics::setHelperMatrices(const RLLKinPoseConfig& eef_pose, RLLInvKinCoeffs* coeffs,
                                                  double* joint_angle_4) const
{
  RLLKinJoints joint_angles_v;
  RLLInvKinHelperMatrices hm;

  Eigen::Vector3d xsw;
  Eigen::Vector3d xsw_n;
  Eigen::Vector3d xwf_n;
  double lsw;
  RLLKinMsg result = shoulderWristVec(eef_pose.pose, &xsw, &xwf_n, &lsw);
  if (result.error())
  {
    return result;
  }

  *joint_angle_4 = joint_angles_v[3] = jointAngle4(lsw, eef_pose.config);

  if (kSmallerThan(*joint_angle_4, lowerJointLimits()(3)) || kGreaterThan(*joint_angle_4, upperJointLimits()(3)))
  {
    return RLLKinMsg::TARGET_TOO_CLOSE;
  }

  joint_angles_v[0] = jointAngle1Virtual(xsw);
  joint_angles_v[1] = jointAngle2Virtual(xsw, lsw, eef_pose.config);

  xsw_n.noalias() = xsw.normalized();                // normalized shoulder to wrist vector
  Eigen::Matrix3d xsw_n_cross = crossMatrix(xsw_n);  // cross product matrix of xsw_n

  // virtual upper arm pose
  RLLKinFrame mbu_v = RLLKinFrame(limbLength(0), joint_angles_v(0), 0.0, -M_PI / 2.0) *
                      RLLKinFrame(0.0, joint_angles_v(1), 0.0, M_PI / 2.0) *
                      RLLKinFrame(limbLength(1), 0.0, 0.0, M_PI / 2.0);

  // helper matrix As, Bs, Cs to rotate mbu_v
  hm.as = xsw_n_cross * mbu_v.ori();
  hm.bs = -xsw_n_cross * hm.as;
  hm.cs = (xsw_n * xsw_n.transpose()) * mbu_v.ori();

  // elbow pose in upper arm coordinates, same as virtual elbow pose
  RLLKinFrame mue = RLLKinFrame(0.0, joint_angles_v(3), 0.0, -M_PI / 2.0);

  // helper matrix Aw, Bw, Cw
  hm.aw = mue.ori().transpose() * hm.as.transpose() * eef_pose.pose.ori();
  hm.bw = mue.ori().transpose() * hm.bs.transpose() * eef_pose.pose.ori();
  hm.cw = mue.ori().transpose() * hm.cs.transpose() * eef_pose.pose.ori();

  coeffs->setHelperMatrices(hm, xsw_n, xwf_n, *joint_angle_4);

  return RLLKinMsg::SUCCESS;
}

RLLKinMsg RLLInverseKinematics::computeFeasibleIntervals(RLLInvKinNsIntervals* intervals) const
{
  return intervals->computeFeasibleIntervals(lowerJointLimits(), upperJointLimits());
}

void RLLInverseKinematics::addRemainingConfigs(const size_t it, const double dist_from_seed,
                                               RLLKinGlobalConfigs* configs)
{
  if (it < configs->size() - 1 || configs->size() == RLL_NUM_GLOBAL_CONFIGS ||
      dist_from_seed < std::numeric_limits<double>::infinity())
  {
    // there are still configurations to check or all configs are already in the list or a usable solution was already
    // found
    return;
  }

  for (uint8_t i = 0; i < RLL_NUM_GLOBAL_CONFIGS; ++i)
  {
    if (std::find(configs->begin(), configs->end(), RLLKinGlobalConfig(i)) == configs->end())
    {
      // config not yet added, so add it
      configs->emplace_back(i);
    }
  }
}

double RLLInverseKinematics::mapArmAngleForGC4(const RLLKinGlobalConfig& seed_config,
                                               const RLLKinGlobalConfig& selected_config, const double seed_arm_angle)
{
  if (!kIsEqual(seed_config.gc4(), selected_config.gc4()))
  {
    // seed elbow config and currently selected config differ, change arm_angle if configurations differ
    // (different reference planes for calculation of arm angle)
    double arm_angle_start = seed_arm_angle + M_PI;
    return mapAngleInPiRange(arm_angle_start);
  }

  return seed_arm_angle;
}
