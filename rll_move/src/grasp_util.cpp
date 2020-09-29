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

#include <rll_move/grasp_util.h>

#include <eigen_conversions/eigen_msg.h>
#include <fcl/collision.h>
#include <fcl/math/vec_3f.h>
#include <fcl/shape/geometric_shapes.h>
#include <tf/tf.h>

// for mesh objects
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

#define ALLOWED_ROTATION_DEVIATION (.01)

CollisionObjectBuilder& CollisionObjectBuilder::setPose(geometry_msgs::Pose pose)
{
  pose_ = pose;
  return *this;
}

// TODO(mark): rename reset or provide beginPrimitive/beginMesh?
CollisionObjectBuilder& CollisionObjectBuilder::begin()
{
  object_ = moveit_msgs::CollisionObject();
  pose_ = geometry_msgs::Pose();
  pose_.orientation.w = 1;  // minimum required pose TODO(mark): point up
  return *this;
}

CollisionObjectBuilder& CollisionObjectBuilder::addMesh(const std::string& file_path, double scale)
{
  // Path where the .dae or .stl object is located
  shapes::Mesh* m = shapes::createMeshFromResource(file_path, { scale, scale, scale });
  if (m == nullptr)
  {
    ROS_FATAL("Your mesh '%s' failed to loaded", file_path.c_str());
    return *this;
  }

  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);  // TODO(mark): who owns this pointer?! should I delete it?
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  // TODO(mark): isn't it super expensive to always copy the mesh around?
  object_.meshes.push_back(mesh);
  is_primitive_ = false;

  return *this;
}

CollisionObjectBuilder& CollisionObjectBuilder::addCylinder(double radius, double height)
{
  shape_msgs::SolidPrimitive primitive;
  primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = height;
  primitive.dimensions[1] = radius;
  object_.primitives.push_back(primitive);
  is_primitive_ = true;
  return *this;
}

CollisionObjectBuilder& CollisionObjectBuilder::positionBottomAtZ(double height, double collision_offset)
{
  pose_.position.z += height / 2 + collision_offset;

  return *this;
}

CollisionObjectBuilder& CollisionObjectBuilder::setPosition(const geometry_msgs::Point& point)
{
  pose_.position = point;
  return *this;
}

CollisionObjectBuilder& CollisionObjectBuilder::translate(double x, double y, double z)
{
  pose_.position.x += x;
  pose_.position.y += y;
  pose_.position.z += z;
  return *this;
}

CollisionObjectBuilder& CollisionObjectBuilder::rotateRPY(double r, double p, double y)
{
  static tf2::Quaternion quaternion;
  quaternion.setRPY(r, p, y);
  pose_.orientation.x = quaternion.x();
  pose_.orientation.y = quaternion.y();
  pose_.orientation.z = quaternion.z();
  pose_.orientation.w = quaternion.w();
  return *this;
}

CollisionObjectBuilder& CollisionObjectBuilder::addBox(double w, double h, double d)
{
  shape_msgs::SolidPrimitive primitive;
  primitive.type = shape_msgs::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = w;
  primitive.dimensions[1] = h;
  primitive.dimensions[2] = d;
  is_primitive_ = true;
  object_.primitives.push_back(primitive);
  return *this;
}

moveit_msgs::CollisionObject CollisionObjectBuilder::build(const std::string& id, const std::string& frame_id)
{
  object_.operation = moveit_msgs::CollisionObject::ADD;
  object_.id = id;
  object_.header.frame_id = frame_id;

  if (is_primitive_)
  {
    object_.primitive_poses.push_back(pose_);
  }
  else
  {
    object_.mesh_poses.push_back(pose_);
  }

  return std::move(object_);  // TODO(mark): want to mark as invalid, but move on return is bad? (see effective c++)
}
