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

#include <rll_kinematics/arm_angle_intervals.h>

bool RLLKinArmAngleInterval::operator<(const RLLKinArmAngleInterval& rhs) const
{
  return lower_limit < rhs.lower_limit;
}

bool RLLInvKinIntervalLimit::operator<(const RLLInvKinIntervalLimit& rhs) const
{
  return arm_angle_ < rhs.arm_angle_;
}

void RLLInvKinNsIntervals::mergeSortedBlockedIntervals()
{
  ArmAngleIntervalCollection sorted_intervals;

  if (blocked_intervals_.empty())
  {
    return;
  }

  sorted_intervals.push_back(blocked_intervals_[0]);

  for (size_t i = 1; i < blocked_intervals_.size(); ++i)
  {
    RLLKinArmAngleInterval back = sorted_intervals.back();

    if (back.upper_limit < blocked_intervals_[i].lower_limit)
    {
      // there is a feasible interval between back and i, so a new blocked interval starts
      sorted_intervals.push_back(blocked_intervals_[i]);
    }
    else if (back.upper_limit < blocked_intervals_[i].upper_limit)
    {
      // intervals overlap, but new interval is larger, extend last sorted interval to upper limit of interval i
      back.upper_limit = blocked_intervals_[i].upper_limit;
      sorted_intervals.back() = back;
    }
  }

  blocked_intervals_ = sorted_intervals;
}

void RLLInvKinNsIntervals::feasibleIntervalsFromBlocked()
{
  if (blocked_intervals_.empty())
  {
    feasible_intervals_.emplace_back(-M_PI, M_PI);
    return;
  }

  if (blocked_intervals_.size() == 1 && kIsEqual(blocked_intervals_[0].lower_limit, -M_PI) &&
      kIsEqual(blocked_intervals_[0].upper_limit, M_PI))
  {
    return;
  }

  if ((blocked_intervals_[0].lower_limit > -M_PI))
  {
    feasible_intervals_.emplace_back(-M_PI, M_PI);
  }

  for (size_t i = 0; i < blocked_intervals_.size(); ++i)
  {
    RLLKinArmAngleInterval blocked_interval = blocked_intervals_[i];

    if (!feasible_intervals_.empty())
    {
      feasible_intervals_.back().upper_limit = blocked_interval.lower_limit;
    }

    if (blocked_interval.upper_limit < M_PI)
    {
      feasible_intervals_.emplace_back(blocked_interval.upper_limit, M_PI);
    }
  }

  if (kIsEqual(feasible_intervals_.back().upper_limit, M_PI) &&
      kIsEqual(feasible_intervals_.front().lower_limit, -M_PI))
  {
    feasible_intervals_.back().overlap = true;
    feasible_intervals_.front().overlap = true;
  }
}

void RLLInvKinNsIntervals::determineBlockedIntervals(const RLLInvKinIntervalLimits& interval_limits)
{
  // classification if blocked or feasible interval depends on derivative of joint angle w.r.t. arm angle (sign)

  size_t size = interval_limits.size();

  for (size_t j = 0; j < size; ++j)
  {
    size_t j_next = (j + 1) % size;  // zero for j = size -1

    if (std::signbit(interval_limits[j].jointAngle()) == std::signbit(interval_limits[j].jointDerivative()) ||
        std::signbit(interval_limits[j_next].jointAngle()) != std::signbit(interval_limits[j_next].jointDerivative()))
    {
      if (j == (size - 1))
      {
        // overlapping interval at the end and the beginning of the arm angle range
        blocked_intervals_.emplace_back(-M_PI, interval_limits[0].armAngle());
        blocked_intervals_.emplace_back(interval_limits[j].armAngle(), M_PI);
      }
      else
      {
        blocked_intervals_.emplace_back(interval_limits[j].armAngle(), interval_limits[j + 1].armAngle());
      }
    }
  }
}

void RLLInvKinNsIntervals::insertLimit(RLLInvKinIntervalLimits* interval_limits, const RLLInvKinCoeffs::JointType type,
                                       const double joint_angle, const double arm_angle, const int index)
{
  // check if calculated arm angle limit is matching to limit in joint-space
  // if this is not the case, the joint angle does not coincide with the joint limits for any arm angle in the [−pi, pi]
  // range whole interval is then either feasible or not
  if (kZero(joint_angle - coeffs_.jointAngle(type, index, arm_angle)))
  {
    // precalculate derivative, needed later to determine blocked intervals
    double joint_derivative = coeffs_.jointDerivative(type, index, arm_angle, joint_angle);

    interval_limits->emplace_back(arm_angle, joint_angle, joint_derivative);
  }
}

void RLLInvKinNsIntervals::mapLimitsToArmAngle(const RLLInvKinCoeffs::JointType type, const double lower_joint_limit,
                                               const double upper_joint_limit, const int index)
{
  RLLInvKinIntervalLimits interval_limits;
  double arm_angle_lower, arm_angle_upper;

  // map lower joint limits to arm angle
  if (coeffs_.armAngleForJointLimit(type, index, lower_joint_limit, &arm_angle_lower, &arm_angle_upper))
  {
    insertLimit(&interval_limits, type, lower_joint_limit, arm_angle_lower, index);
    insertLimit(&interval_limits, type, lower_joint_limit, arm_angle_upper, index);
  }

  // map upper joint limits to arm angle
  if (coeffs_.armAngleForJointLimit(type, index, upper_joint_limit, &arm_angle_lower, &arm_angle_upper))
  {
    insertLimit(&interval_limits, type, upper_joint_limit, arm_angle_lower, index);
    insertLimit(&interval_limits, type, upper_joint_limit, arm_angle_upper, index);
  }

  if (interval_limits.empty())
  {
    // any arm angle can be used to check if the whole interval is feasible or not
    double arm_angle_test = coeffs_.jointAngle(type, index, 0.0);
    if (arm_angle_test >= upper_joint_limit || arm_angle_test <= lower_joint_limit)
    {
      // all angles blocked
      blocked_intervals_.emplace_back(-M_PI, M_PI);
    }

    return;
  }

  std::sort(interval_limits.begin(), interval_limits.end());
  determineBlockedIntervals(interval_limits);
}

void RLLInvKinNsIntervals::closestFeasibleintervalForArmAngle(const int index, const double arm_angle,
                                                              RLLKinArmAngleInterval* current_interval) const
{
  RLLKinArmAngleInterval closest_interval;
  if (index > 0)
  {  // arm angle between two feasible interval and upper interval is at index, set to middle of closest interval
    double middle_upper_interval =
        (feasible_intervals_[index].upper_limit + feasible_intervals_[index].lower_limit) / 2;
    double middle_lower_interval =
        (feasible_intervals_[index - 1].upper_limit + feasible_intervals_[index - 1].lower_limit) / 2;

    if (middle_upper_interval - arm_angle <= arm_angle - middle_lower_interval)
    {
      closest_interval.lower_limit = middle_upper_interval;
    }
    else
    {
      closest_interval.lower_limit = middle_lower_interval;
    }
  }
  else
  {  // arm angle either below first feasible interval or above last feasible interval
    double middle_first_interval =
        (feasible_intervals_.front().upper_limit + feasible_intervals_.front().lower_limit) / 2;
    double middle_last_interval = (feasible_intervals_.back().upper_limit + feasible_intervals_.back().lower_limit) / 2;

    if (index == 0)
    {  // arm angle in overlap region below lowest feasible angle
      if (middle_first_interval - arm_angle <= ((arm_angle + M_PI) + (M_PI - middle_last_interval)))
      {
        closest_interval.lower_limit = middle_first_interval;
      }
      else
      {
        closest_interval.lower_limit = middle_last_interval;
      }
    }
    else
    {  // arm angle greater than last feasible angle
      if (arm_angle - middle_last_interval <= ((M_PI - arm_angle) + (middle_first_interval + M_PI)))
      {
        closest_interval.lower_limit = middle_last_interval;
      }
      else
      {
        closest_interval.lower_limit = middle_first_interval;
      }
    }
  }

  // only one arm angle admissible at this point
  closest_interval.upper_limit = closest_interval.lower_limit;
  *current_interval = closest_interval;
}

RLLKinMsg RLLInvKinNsIntervals::intervalForArmAngle(double* arm_angle, RLLKinArmAngleInterval* current_interval) const
{
  bool interval_found = false;
  int index = -1;

  if (feasible_intervals_.empty())
  {
    return RLLKinMsg::NO_SOLUTION_FOR_ARMANGLE;
  }

  for (size_t i = 0; i < feasible_intervals_.size(); ++i)
  {
    if (*arm_angle <= feasible_intervals_[i].upper_limit)
    {
      if (*arm_angle >= feasible_intervals_[i].lower_limit)
      {
        interval_found = true;
        *current_interval = feasible_intervals_[i];
      }

      // Arm angle is in a blocked interval below the feasible interval i, so no need to keep searching.
      // Save the interval index to speed up the closest feasible interval search later on.
      index = i;
      break;
    }
  }

  if (interval_found)
  {
    if (!current_interval->overlap ||
        (kIsEqual(current_interval->lower_limit, -M_PI) && kIsEqual(current_interval->upper_limit, M_PI)))
    {
      return RLLKinMsg::SUCCESS;
    }

    if (kIsEqual(current_interval->lower_limit, -M_PI))
    {
      // overlapping at -M_PI, map everything smaller as or equal the upper limit of the first interval to the
      // [pi, 3*pi] range and take the lower limit of the last interval as lower limit
      if (*arm_angle < current_interval->upper_limit)
      {
        *arm_angle += 2 * M_PI;
      }
      current_interval->upper_limit += 2 * M_PI;
      current_interval->lower_limit = feasible_intervals_.back().lower_limit;
    }
    else if (kIsEqual(current_interval->upper_limit, M_PI))
    {
      // overlapping at -M_PI, map everything smaller as or equal the upper limit of the first interval to the
      // [pi, 3*pi] range and keep the lower limit of the current interval
      current_interval->upper_limit = 2 * M_PI + feasible_intervals_.front().upper_limit;
      if (*arm_angle < feasible_intervals_.front().upper_limit)
      {
        *arm_angle += 2 * M_PI;
      }
    }

    return RLLKinMsg::SUCCESS;
  }

  closestFeasibleintervalForArmAngle(index, *arm_angle, current_interval);

  return RLLKinMsg::SUCCESS;
}

RLLKinMsg RLLInvKinNsIntervals::computeFeasibleIntervals(const RLLKinJoints& lower_joint_limits,
                                                         const RLLKinJoints& upper_joint_limits)
{
  const double MARGIN_SINGULARITY = 10 * ZERO_ROUNDING_TOL;

  for (uint8_t i = 0; i < RLL_NUM_JOINTS_P; ++i)
  {
    double psi_singular;
    if (coeffs_.pivotSingularity(i, &psi_singular))
    {
      // blocked interval due to singularity at psi_singular
      blocked_intervals_.emplace_back(psi_singular - MARGIN_SINGULARITY, psi_singular + MARGIN_SINGULARITY);
    }

    mapLimitsToArmAngle(RLLInvKinCoeffs::PIVOT_JOINT, lower_joint_limits(2 * i), upper_joint_limits(2 * i), i);
  }

  for (size_t i = 0; i < RLL_NUM_JOINTS_H; ++i)
  {
    mapLimitsToArmAngle(RLLInvKinCoeffs::HINGE_JOINT, lower_joint_limits(4 * i + 1), upper_joint_limits(4 * i + 1), i);
  }

  std::sort(blocked_intervals_.begin(), blocked_intervals_.end());
  mergeSortedBlockedIntervals();
  feasibleIntervalsFromBlocked();

  return RLLKinMsg::SUCCESS;
}
