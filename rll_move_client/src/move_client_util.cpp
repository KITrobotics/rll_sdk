/*
 * This file is part of the Robot Learning Lab Move Client
 *
 * Copyright (C) 2019 Mark Weinreuter <uieai@student.kit.edu>
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

#include <cmath>
#include <rll_move_client/util.h>

void orientationFromRPY(double roll, double pitch, double yaw, geometry_msgs::Quaternion* const orientation)
{
  // can share the instance across calls
  static tf2::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);
  copyQuaternion(quaternion, orientation);
}

void copyQuaternion(const tf2::Quaternion& quaternion, geometry_msgs::Quaternion* const orientation)
{
  orientation->x = quaternion.x();
  orientation->y = quaternion.y();
  orientation->z = quaternion.z();
  orientation->w = quaternion.w();
}

bool compareJointValues(const std::vector<double>& joint_values1, const std::vector<double>& joint_values2,
                        double tolerance)
{
  if (joint_values1.size() != joint_values2.size())
  {
    return false;
  }

  auto iter1 = joint_values1.begin();
  auto iter2 = joint_values2.begin();

  for (; iter1 != joint_values1.end() && iter2 != joint_values2.end(); ++iter1, ++iter2)
  {
    if (std::abs(*iter1 - *iter2) > tolerance)
    {
      return false;
    }
  }
  return true;
}
