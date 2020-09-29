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

#ifndef RLL_MOVE_GRASP_UTIL_H
#define RLL_MOVE_GRASP_UTIL_H

#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>

class CollisionObjectBuilder
{
public:
  static constexpr double DEFAULT_Z_OFFSET = 0.002;

  // TODO(mark): rename reset or provide beginPrimitive/beginMesh?
  CollisionObjectBuilder& begin();

  // TODO(mark): make these methods only callable for r-value references && to force users to chain them?
  CollisionObjectBuilder& setPose(geometry_msgs::Pose pose);

  CollisionObjectBuilder& addMesh(const std::string& file_path, double scale = 0.01);

  CollisionObjectBuilder& addCylinder(double radius, double height);

  // TODO(mark): Add methods to place object relative to their BBox sides, e.g. bottom(0) => place bottom of BBOX at 0
  CollisionObjectBuilder& positionBottomAtZ(double height, double collision_offset = DEFAULT_Z_OFFSET);

  CollisionObjectBuilder& setPosition(const geometry_msgs::Point& point);

  CollisionObjectBuilder& translate(double x, double y, double z);

  CollisionObjectBuilder& rotateRPY(double r, double p, double y);

  CollisionObjectBuilder& addBox(double w, double h, double d);

  moveit_msgs::CollisionObject build(const std::string& id, const std::string& frame_id);

private:
  moveit_msgs::CollisionObject object_;
  bool is_primitive_ = true;
  geometry_msgs::Pose pose_;
};

#endif  // RLL_MOVE_GRASP_UTIL_H
