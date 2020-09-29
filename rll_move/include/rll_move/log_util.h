/*
 * This file is part of the Robot Learning Lab SDK
 *
 * Copyright (C) 2020 Mark Weinreuter <mark@student.kit.edu>
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

#ifndef RLL_MOVE_LOG_UTIL_H_
#define RLL_MOVE_LOG_UTIL_H_

// MACROS to easily debug ROS types
#define ROS_INFO_VEC(name, vec) ROS_INFO("%s: %.3f, %.3f, %.3f", (name), (vec).x(), (vec).y(), (vec).z())
#define ROS_INFO_POSE(name, pose)                                                                                      \
  ROS_INFO("%s: p=(%.2f, %.2f, %.2f) q=(%.2f, %.2f, %.2f, %.2f)", (name), (pose).position.x, (pose).position.y,        \
           (pose).position.z, (pose).orientation.x, (pose).orientation.y, (pose).orientation.z, (pose).orientation.w)

#define ROS_INFO_POS(name, position) ROS_INFO("%s: %.2f, %.2f, %.2f", (name), (position).x, (position).y, (position).z)
#define ROS_INFO_POS_IDX(name, position)                                                                               \
  ROS_INFO("%s: %.2f, %.2f, %.2f", (name), (position)[0], (position)[1], (position)[2])

#endif /* RLL_MOVE_LOG_UTIL_H_ */
