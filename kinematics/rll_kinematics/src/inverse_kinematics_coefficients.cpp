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

#include <rll_kinematics/inverse_kinematics_coefficients.h>

void RLLInvKinCoeffs::setHelperMatrices(const RLLInvKinHelperMatrices& hm, const Eigen::Vector3d& xsw_n,
                                        const Eigen::Vector3d& xwf_n, const double joint_angle_4)
{
  an_[0] = hm.as(1, 1);
  bn_[0] = hm.bs(1, 1);
  cn_[0] = hm.cs(1, 1);
  ad_[0] = hm.as(0, 1);
  bd_[0] = hm.bs(0, 1);
  cd_[0] = hm.cs(0, 1);

  an_[1] = -hm.as(2, 2);
  bn_[1] = -hm.bs(2, 2);
  cn_[1] = -hm.cs(2, 2);
  ad_[1] = -hm.as(2, 0);
  bd_[1] = -hm.bs(2, 0);
  cd_[1] = -hm.cs(2, 0);

  an_[2] = hm.aw(1, 2);
  bn_[2] = hm.bw(1, 2);
  cn_[2] = hm.cw(1, 2);
  ad_[2] = hm.aw(0, 2);
  bd_[2] = hm.bw(0, 2);
  cd_[2] = hm.cw(0, 2);

  an_[3] = hm.aw(2, 1);
  bn_[3] = hm.bw(2, 1);
  cn_[3] = hm.cw(2, 1);
  ad_[3] = -hm.aw(2, 0);
  bd_[3] = -hm.bw(2, 0);
  cd_[3] = -hm.cw(2, 0);

  a_[0] = hm.as(2, 1);
  b_[0] = hm.bs(2, 1);
  c_[0] = hm.cs(2, 1);
  a_[1] = hm.aw(2, 2);
  b_[1] = hm.bw(2, 2);
  c_[1] = hm.cw(2, 2);

  sw_.xsw_n = xsw_n;
  sw_.xwf_n = xwf_n;

  joint_angle_4_ = joint_angle_4;
}

void RLLInvKinCoeffs::setGC(const RLLKinGlobalConfig& config)
{
  // shoulder
  gc_p_[0] = config.gc2();
  gc_p_[1] = config.gc2();
  gc_h_[0] = config.gc2();

  // wrist
  gc_p_[2] = config.gc6();
  gc_p_[3] = config.gc6();
  gc_h_[1] = config.gc6();
}

void RLLInvKinCoeffs::preparePivotDerivatives()
{
  // some pivot joint derivative coefficients can be precalculated
  for (uint8_t i = 0; i < RLL_NUM_JOINTS_P; ++i)
  {
    // these do not contain gc (blunder in paper)
    at_[i] = cn_[i] * bd_[i] - bn_[i] * cd_[i];
    bt_[i] = an_[i] * cd_[i] - cn_[i] * ad_[i];
    ct_[i] = an_[i] * bd_[i] - bn_[i] * ad_[i];
  }
}

double RLLInvKinCoeffs::jointAngle(const JointType type, const uint8_t i, const double arm_angle) const
{
  switch (type)
  {
    case PIVOT_JOINT:
      return jointAnglePivot(i, arm_angle);
    case HINGE_JOINT:
      return jointAngleHinge(i, arm_angle);
  }
}

double RLLInvKinCoeffs::jointAnglePivot(const uint8_t i, const double arm_angle) const
{
  return atan2(gc_p_[i] * (an_[i] * sin(arm_angle) + bn_[i] * cos(arm_angle) + cn_[i]),
               gc_p_[i] * (ad_[i] * sin(arm_angle) + bd_[i] * cos(arm_angle) + cd_[i]));
}

double RLLInvKinCoeffs::jointAngleHinge(const uint8_t i, const double arm_angle) const
{
  return gc_h_[i] * kAcos(a_[i] * sin(arm_angle) + b_[i] * cos(arm_angle) + c_[i]);
}

double RLLInvKinCoeffs::jointDerivative(const JointType type, const uint8_t i, const double arm_angle,
                                        const double joint_angle) const
{
  switch (type)
  {
    case PIVOT_JOINT:
      return jointDerivativePivot(i, arm_angle);
    case HINGE_JOINT:
      return jointDerivativeHinge(i, arm_angle, joint_angle);
  }
}

double RLLInvKinCoeffs::jointDerivativePivot(const uint8_t i, const double arm_angle) const
{
  double u = (an_[i] * sin(arm_angle) + bn_[i] * cos(arm_angle) + cn_[i]);
  double v = (ad_[i] * sin(arm_angle) + bd_[i] * cos(arm_angle) + cd_[i]);

  return (at_[i] * sin(arm_angle) + bt_[i] * cos(arm_angle) + ct_[i]) / (pow(u, 2) + pow(v, 2));
}

double RLLInvKinCoeffs::jointDerivativeHinge(const uint8_t i, const double arm_angle, const double joint_angle) const
{
  // fabs is needed because sin(joint_angle) is actually sqrt(1 - pow(cos(joint_angle), 2))
  return -gc_h_[i] *
         ((a_[i] * cos(arm_angle) - b_[i] * sin(arm_angle)) /
          fabs(sin(joint_angle)));  // division by zero impossible, function is only called in joint limit != 0 or Pi.
}

bool RLLInvKinCoeffs::armAngleForJointLimit(const JointType type, const uint8_t i, const double joint_angle,
                                            double* arm_angle_lower, double* arm_angle_upper) const
{
  bool success;

  switch (type)
  {
    case PIVOT_JOINT:
      success = armAnglePivot(i, joint_angle, arm_angle_lower, arm_angle_upper);
      break;
    case HINGE_JOINT:
      success = armAngleHinge(i, joint_angle, arm_angle_lower, arm_angle_upper);
      break;
  }

  if (success && kIsEqual(*arm_angle_lower, *arm_angle_upper))
  {
    // Arm angle is at a singular position for hinge joints. The arm angle is defined at this position, but not the
    // derivative. A single arm angle corresponds to the joint angle and the arm angle is either a global minimum or
    // maximum. We still return false here because the arm angle interval only touches the limit here and it is not a
    // crossing point.
    return false;
  }

  return success;
}

bool RLLInvKinCoeffs::armAnglePivot(const uint8_t i, const double joint_angle, double* arm_angle_lower,
                                    double* arm_angle_upper) const
{
  double ap = gc_p_[i] * ((cd_[i] - bd_[i]) * tan(joint_angle) + (bn_[i] - cn_[i]));
  double bp = 2 * gc_p_[i] * (ad_[i] * tan(joint_angle) - an_[i]);
  double cp = gc_p_[i] * ((bd_[i] + cd_[i]) * tan(joint_angle) - (bn_[i] + cn_[i]));
  double discriminant = pow(bp, 2) - 4 * ap * cp;

  if (!kGreaterZero(discriminant))
  {
    // joint angle is not reached in null space for current pose / configuration
    return false;
  }

  double sqrt_discr = kSqrt(discriminant);

  *arm_angle_lower = 2 * atan((-bp - sqrt_discr) / (2 * ap));
  *arm_angle_upper = 2 * atan((-bp + sqrt_discr) / (2 * ap));

  return true;
}

bool RLLInvKinCoeffs::armAngleHinge(const uint8_t i, const double joint_angle, double* arm_angle_lower,
                                    double* arm_angle_upper) const
{
  double discriminant = pow(a_[i], 2) + pow(b_[i], 2) - pow(c_[i] - cos(joint_angle), 2);

  if (!kGreaterZero(discriminant))
  {
    // joint angle is not reached in null space for current pose / configuration
    return false;
  }

  double sqrt_discr = kSqrt(discriminant);
  double denominator = cos(joint_angle) + b_[i] - c_[i];

  *arm_angle_lower = 2 * atan((a_[i] - sqrt_discr) / denominator);
  *arm_angle_upper = 2 * atan((a_[i] + sqrt_discr) / denominator);

  return true;
}

bool RLLInvKinCoeffs::pivotSingularity(const uint8_t i, double* value) const
{
  double discriminant = pow(at_[i], 2) + pow(bt_[i], 2) - pow(ct_[i], 2);

  if (kGreaterZero(discriminant) && kZero(kSqrt(discriminant)))
  {
    *value = 2 * atan(at_[i] / (bt_[i] - ct_[i]));
    return true;
  }

  return false;
}
