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

#ifndef RLL_MOVE_GRASP_OBJECT_H
#define RLL_MOVE_GRASP_OBJECT_H
#include <ros/ros.h>

#include <fcl/math/vec_3f.h>
#include <fcl/shape/geometric_shapes.h>
#include <moveit_msgs/CollisionObject.h>
#include <rll_move/log_util.h>
#include <tf/tf.h>
#include <utility>
#include <vector>

struct GraspPose
{
  geometry_msgs::Pose eef_pose, object_pose;
};

/**
 * Abstract base class for any object to be grasped. Each object keeps track of its initial and last placement and grip
 * pose to ensure it can be reset into its initial position. Each object corresponds to a Moveit CollisionObject which
 * is used for collision checking during motion planning. Accordingly, a GraspObject retrieves its values/pose from the
 * underlying CollisionObject. Additionally each GraspObject is described by a (rough) bounding collision geometry which
 * is used for coarse collision checking, e.g. whether a GraspObject is inside a DropArea. The full Moveit collision
 * representation of the CollisionObject is not used, since this representation may be too complex, e.g., composed of
 * multiple shapes or meshes, meant for precise collision checks.
 */
class GraspObject
{
public:
  explicit GraspObject() = default;

  virtual ~GraspObject() = default;

  /**
   * Determine whether the object can be released for the given gripper pose, but validating the object's placement
   * constraints.
   */
  bool canBeReleased(const geometry_msgs::Pose& eef_target_pose) const;

  /**
   * Determine whether the object can be released for the given gripper pose, but validating the object's placement
   * constraints.
   */
  bool canBeGrasped(const geometry_msgs::Pose& eef_target_pose) const;

  /**
   * Once the object has been grasped store the gripper and object pose used to grip the object.
   */
  void hasBeenGrasped(const geometry_msgs::Pose& eef_pose, const geometry_msgs::Pose& unattached_object_pose);

  /**
   * Once the object has been released store the gripper and object pose used to place the object.
   */
  void hasBeenReleased(const geometry_msgs::Pose& eef_pose, const geometry_msgs::Pose& unattached_object_pose);

  /**
   * Update the underlying collision object. The update is propagated to subclass via onCollisionObjectUpdated().
   */
  void updateDataFromCollisionObject(const moveit_msgs::CollisionObject& collision_object)
  {
    if (!hasBeenUpdatedFromCollisionObject())
    {
      id_ = collision_object.id;
      ROS_INFO("GraspObject %s updated for the first time", id_.c_str());
    }
    else if (id_ != collision_object.id)
    {
      ROS_FATAL("Trying to update the GraspObject %s with from CollisionObject with different id: %s", id_.c_str(),
                collision_object.id.c_str());
      return;
    }
    collision_object_ = collision_object;
    onUpdateDataFromCollisionObject(collision_object);
  }

  /**
   * Return the center pose of the whole collision object, e.g. the center of the bounding box.
   */
  virtual const geometry_msgs::Pose& getCenterPose() const = 0;

  // TODO(mark): return a Shape message rather than a fcl object?
  /**
   * The approximated collision geometry to be used in rough collision checks.
   */
  virtual const fcl::CollisionGeometry* getBoundingCollisionGeometry() const = 0;

  bool pointInCollisionGeometry(const geometry_msgs::Pose& obj_target_pose, const geometry_msgs::Point& point) const;

  // TODO(mark): could be automatically retrieved from the collision geometry
  /**
   * Dimensions of the bounding box for this GraspObject.
   */
  virtual const geometry_msgs::Vector3& getBoundingBoxDimensions() const = 0;

  bool hasBeenMoved() const
  {
    return initial_grip_set_;
  }

  const moveit_msgs::CollisionObject& getCollisionObject() const
  {
    return collision_object_;
  }

  /**
   * Given the EEf pose return the objects pose by applying the EEF transform to the object.
   * If the object is not attached, its untransformed pose is returned.
   */
  geometry_msgs::Pose getObjectPoseForEEFPose(const geometry_msgs::Pose& eef_pose) const;

  /**
   * Returns the ID of this object, it is also the ID of the underlying moveit CollisionObject.
   */
  const std::string& getID() const
  {
    if (id_.empty())
    {
      ROS_FATAL("Using GraspObject that has not been initialized.");
    }
    return id_;
  }

  bool isCurrentlyAttached() const
  {
    return is_currently_attached_;
  }

  bool hasBeenUpdatedFromCollisionObject()
  {
    return !id_.empty();
  }

private:
  std::string id_;
  GraspPose last_placement_, initial_placement_;
  bool is_currently_attached_ = false, initial_grip_set_ = false;
  moveit_msgs::CollisionObject collision_object_;

  virtual void onUpdateDataFromCollisionObject(const moveit_msgs::CollisionObject& collision_object) = 0;
};

/*
 * A grasp object that is described by a single (Moveit) collision primitive, e.g. a box or cylinder.
 */
class SinglePrimitiveGraspObject : public GraspObject
{
public:
  explicit SinglePrimitiveGraspObject(uint8_t type) : type_(type)
  {
  }

  const geometry_msgs::Vector3& getBoundingBoxDimensions() const override
  {
    return box_dimensions_;
  }

  const geometry_msgs::Pose& getCenterPose() const override
  {
    return pose_;
  }

protected:
  // keep track of the collision objects type, pose and size in dedicated members
  geometry_msgs::Pose pose_;
  std::vector<double> dimensions_;
  geometry_msgs::Vector3 box_dimensions_;
  void copyDataFromCollisionObject(const moveit_msgs::CollisionObject& collision_object);

private:
  uint8_t type_;  // NOLINT
};

class SingleMeshGraspObject : public GraspObject
{
public:
  explicit SingleMeshGraspObject(const geometry_msgs::Vector3& dim) : SingleMeshGraspObject(dim.x, dim.y, dim.z)
  {
  }

  SingleMeshGraspObject(double width, double height, double depth) : box_{ width, height, depth }
  {
    updateBBox();
  }

  const geometry_msgs::Vector3& getBoundingBoxDimensions() const override
  {
    return box_dimensions_;
  }

  const geometry_msgs::Pose& getCenterPose() const override
  {
    return pose_;
  }

  const fcl::CollisionGeometry* getBoundingCollisionGeometry() const override
  {
    return &box_;
  }

private:
  void onUpdateDataFromCollisionObject(const moveit_msgs::CollisionObject& collision_object) override;

  void updateBBox()
  {
    // TODO(mark): verify dimensions in right order
    box_dimensions_.y = box_.side[0];
    box_dimensions_.x = box_.side[1];
    box_dimensions_.z = box_.side[2];
  }

  // TOOD(mark): store pointer to collision geometry to allow different shapes
  fcl::Box box_;
  geometry_msgs::Pose pose_;
  geometry_msgs::Vector3 box_dimensions_;
};

class BoxGraspObject : public SinglePrimitiveGraspObject
{
public:
  explicit BoxGraspObject() : SinglePrimitiveGraspObject(shape_msgs::SolidPrimitive::BOX)
  {
  }

  const fcl::CollisionGeometry* getBoundingCollisionGeometry() const override
  {
    ROS_INFO_STREAM("FCL_Box: side=" << fcl_box_.side);

    return &fcl_box_;
  }

private:
  fcl::Box fcl_box_;

  void onUpdateDataFromCollisionObject(const moveit_msgs::CollisionObject& collision_object) override;
};

class CylinderGraspObject : public SinglePrimitiveGraspObject
{
public:
  explicit CylinderGraspObject() : SinglePrimitiveGraspObject(shape_msgs::SolidPrimitive::CYLINDER), fcl_cylinder_(0, 0)
  {
  }

  const fcl::CollisionGeometry* getBoundingCollisionGeometry() const override
  {
    ROS_INFO("FCL_Cylinder: r=%.3f, lz=%.3f", fcl_cylinder_.radius, fcl_cylinder_.lz);
    return &fcl_cylinder_;
  }

private:
  float radius_ = 0, height_ = 0;
  fcl::Cylinder fcl_cylinder_;

  void onUpdateDataFromCollisionObject(const moveit_msgs::CollisionObject& collision_object) override;
};

#endif  // RLL_MOVE_GRASP_OBJECT_H
