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

#ifndef RLL_MOVE_CLIENT_UTIL_H
#define RLL_MOVE_CLIENT_UTIL_H

#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>

void orientationFromRPY(double roll, double pitch, double yaw, geometry_msgs::Quaternion* orientation);
void copyQuaternion(const tf2::Quaternion& quaternion, geometry_msgs::Quaternion* orientation);

bool compareJointValues(const std::vector<double>& joint_values1, const std::vector<double>& joint_values2,
                        double tolerance = 1.e-4);

#endif  // RLL_MOVE_CLIENT_UTIL_H
