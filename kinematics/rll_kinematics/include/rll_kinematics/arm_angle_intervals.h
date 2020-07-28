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

#ifndef RLL_KINEMATICS_ARM_ANGLE_INTERVALS_H
#define RLL_KINEMATICS_ARM_ANGLE_INTERVALS_H

#include <rll_kinematics/inverse_kinematics_coefficients.h>

class RLLInvKinIntervalLimit
{
public:
  RLLInvKinIntervalLimit() = default;

  RLLInvKinIntervalLimit(const double arm_angle, const double joint_angle, const double joint_derivative)
    : arm_angle_(arm_angle), joint_angle_(joint_angle), joint_derivative_(joint_derivative)
  {
  }

  double armAngle() const
  {
    return arm_angle_;
  }

  double jointAngle() const
  {
    return joint_angle_;
  }

  double jointDerivative() const
  {
    return joint_derivative_;
  }

  bool operator<(const RLLInvKinIntervalLimit& rhs) const;

private:
  double arm_angle_;
  double joint_angle_;
  double joint_derivative_;
};

using RLLInvKinIntervalLimits = boost::container::static_vector<RLLInvKinIntervalLimit, 4>;

class RLLKinArmAngleInterval : public RLLKinematicsBase
{
public:
  RLLKinArmAngleInterval() = default;

  RLLKinArmAngleInterval(const double lower, const double upper)
  {
    setLimits(lower, upper);
  }

  void setLimits(double lower, double upper);
  void setLowerLimit(double lower);
  void setUpperLimit(double upper);
  // compares lower_limit
  bool operator<(const RLLKinArmAngleInterval& rhs) const;

  double lowerLimit() const
  {
    return lower_limit_;
  }

  double upperLimit() const
  {
    return upper_limit_;
  }

  bool overlapping() const
  {
    return overlap_;
  }

private:
  // TODO(wolfgang): use a custom data type for the limits that has an enum specifying the limit type (singularity,
  //                 joint limit)
  double lower_limit_ = 0.0;
  double upper_limit_ = 0.0;
  bool overlap_ = false;  // determines if interval is overlapping from Pi to -Pi
};

class RLLInvKinNsIntervals : public RLLKinematicsBase
{
public:
  explicit RLLInvKinNsIntervals(RLLInvKinCoeffs coeffs) : coeffs_(std::move(coeffs))
  {
  }

  RLLKinMsg computeFeasibleIntervals(const RLLKinJoints& lower_joint_limits, const RLLKinJoints& upper_joint_limits);
  RLLKinMsg intervalForArmAngle(double* query_arm_angle, RLLKinArmAngleInterval* current_interval,
                                double* fallback_arm_angle) const;

private:
  void mergeSortedBlockedIntervals();
  void feasibleIntervalsFromBlocked();

  void mapLimitsToArmAngle(RLLInvKinCoeffs::JointType type, double lower_joint_limit, double upper_joint_limit,
                           int index);
  void determineBlockedIntervals(const RLLInvKinIntervalLimits& interval_limits);
  void insertLimit(RLLInvKinIntervalLimits* interval_limits, RLLInvKinCoeffs::JointType type, double joint_angle,
                   double arm_angle, int index);

  // used when query arm angle is in a blocked interval, sets fallback arm angle to middle of closest feasible interval
  RLLKinMsg closestFeasibleArmAngle(int index, double query_arm_angle, double* fallback_arm_angle) const;

  RLLInvKinCoeffs coeffs_;
  RLLKinJoints lower_joint_limits_;
  RLLKinJoints upper_joint_limits_;

  // 34 blocked intervals possible in total: 4*5 + 2*5 due to joint limits for pivot and hinge joints, then additionally
  // 4 for pivot joint singularities
  using ArmAngleIntervalCollection = boost::container::static_vector<RLLKinArmAngleInterval, 34>;
  ArmAngleIntervalCollection feasible_intervals_;
  ArmAngleIntervalCollection blocked_intervals_;
};

#endif  // RLL_KINEMATICS_ARM_ANGLE_INTERVALS_H
