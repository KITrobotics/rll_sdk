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

#include <eigen_conversions/eigen_msg.h>
#include <fcl/collision.h>
#include <rll_move/grasp_object.h>

void GraspObject::hasBeenGrasped(const geometry_msgs::Pose& eef_pose, const geometry_msgs::Pose& unattached_object_pose)
{
  is_currently_attached_ = true;
  if (!initial_grip_set_)
  {
    initial_placement_.eef_pose = eef_pose;
    initial_placement_.object_pose = unattached_object_pose;
    initial_grip_set_ = true;
  }
}

void GraspObject::hasBeenReleased(const geometry_msgs::Pose& eef_pose,
                                  const geometry_msgs::Pose& unattached_object_pose)
{
  is_currently_attached_ = false;
  last_placement_.eef_pose = eef_pose;
  last_placement_.object_pose = unattached_object_pose;
}

bool GraspObject::canBeReleased(const geometry_msgs::Pose& /*eef_target_pose*/) const
{
  // geometry_msgs::Pose obj_target_pose = getObjectPoseForEEFPose(eef_target_pose);
  // TODO(mark): implement logic
  return true;
}

bool GraspObject::canBeGrasped(const geometry_msgs::Pose& /*eef_target_pose*/) const
{
  // geometry_msgs::Pose obj_target_pose = getObjectPoseForEEFPose(eef_target_pose);
  // TODO(mark): implement logic
  return true;
}

bool GraspObject::pointInCollisionGeometry(const geometry_msgs::Pose& obj_target_pose,
                                           const geometry_msgs::Point& point) const
{
  fcl::Vec3f object_target_pos(obj_target_pose.position.x, obj_target_pose.position.y, obj_target_pose.position.z);
  fcl::Vec3f test_point(point.x, point.y, point.z);

  fcl::Quaternion3f object_orientation(obj_target_pose.orientation.w, obj_target_pose.orientation.x,
                                       obj_target_pose.orientation.y, obj_target_pose.orientation.z);

  fcl::Transform3f object_tf(object_orientation, object_target_pos);
  fcl::Transform3f point_tf(test_point);

  auto sphere_geometry = fcl::Sphere(.01);

  fcl::CollisionRequest req(1, false);  // find one collision, don't store details
  fcl::CollisionResult res;
  size_t collision_points =
      fcl::collide(getBoundingCollisionGeometry(), object_tf, &sphere_geometry, point_tf, req, res);

  return (collision_points > 0);
}

geometry_msgs::Pose GraspObject::getObjectPoseForEEFPose(const geometry_msgs::Pose& eef_pose) const
{
  geometry_msgs::Pose object_pose = getCenterPose();
  if (is_currently_attached_)
  {
    tf::Transform world_eef, eef_to_object;
    // the object is attached -> object pose is relative to the EEF pose
    tf::poseMsgToTF(object_pose, eef_to_object);
    tf::poseMsgToTF(eef_pose, world_eef);
    tf::Transform world_object = world_eef * eef_to_object;
    tf::poseTFToMsg(world_object, object_pose);
  }
  return object_pose;
}

void SinglePrimitiveGraspObject::copyDataFromCollisionObject(const moveit_msgs::CollisionObject& collision_object)
{
  ROS_ASSERT(collision_object.primitive_poses.size() == 1);
  ROS_ASSERT(collision_object.primitives.size() == 1);
  ROS_ASSERT(collision_object.primitives[0].type == type_);

  // copy the relevant info for direct access
  pose_ = collision_object.primitive_poses[0];
  dimensions_ = collision_object.primitives[0].dimensions;
}

void SingleMeshGraspObject::onUpdateDataFromCollisionObject(const moveit_msgs::CollisionObject& collision_object)
{
  ROS_ASSERT(collision_object.mesh_poses.size() == 1);
  ROS_ASSERT(collision_object.meshes.size() == 1);

  // copy the relevant info for direct access
  pose_ = collision_object.mesh_poses[0];
}

void BoxGraspObject::onUpdateDataFromCollisionObject(const moveit_msgs::CollisionObject& collision_object)
{
  SinglePrimitiveGraspObject::copyDataFromCollisionObject(collision_object);
  ROS_ASSERT(dimensions_.size() == 3);

  // TODO(mark): do we need the box dimensions at all??
  box_dimensions_.x = dimensions_[0];
  box_dimensions_.y = dimensions_[1];
  box_dimensions_.z = dimensions_[2];

  fcl_box_.side[0] = dimensions_[0];
  fcl_box_.side[1] = dimensions_[1];
  fcl_box_.side[2] = dimensions_[2];
}

void CylinderGraspObject::onUpdateDataFromCollisionObject(const moveit_msgs::CollisionObject& collision_object)
{
  SinglePrimitiveGraspObject::copyDataFromCollisionObject(collision_object);
  ROS_ASSERT(dimensions_.size() == 2);

  radius_ = dimensions_[1];
  height_ = dimensions_[0];
  fcl_cylinder_.radius = radius_;
  fcl_cylinder_.lz = height_;

  box_dimensions_.x = radius_ * 2;
  box_dimensions_.y = radius_ * 2;
  box_dimensions_.z = height_;
}
