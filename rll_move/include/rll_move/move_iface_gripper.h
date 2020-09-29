/*
 * This file is part of the Robot Learning Lab SDK
 *
 * Copyright (C) 2018-2019 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
 * Copyright (C) 2019-2020 Mark Weinreuter <uieai@student.kit.edu>
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

#ifndef RLL_MOVE_MOVE_IFACE_GRIPPER_H
#define RLL_MOVE_MOVE_IFACE_GRIPPER_H

#include <ros/ros.h>

#include <fcl/math/vec_3f.h>
#include <fcl/shape/geometric_shapes.h>
#include <memory>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <rll_move/move_iface_error.h>
#include <rll_msgs/MoveGripper.h>
#include <rll_msgs/PickPlace.h>
#include <rll_msgs/PickPlaceHere.h>
#include <rll_msgs/ValidatePickPlace.h>
#include <tf/tf.h>
#include <unordered_map>
#include <vector>

#include "rll_move/grasp_object.h"
#include "rll_move/move_iface_services.h"

class RLLMoveIfaceGripperServices : public virtual RLLMoveIfaceServices
{
public:
  static const std::string MOVE_GRIPPER_SRV_NAME;
  static const std::string PICK_PLACE_SRV_NAME;
  static const std::string PICK_PLACE_HERE_SRV_NAME;
  static const std::string VALIDATE_PICK_PLACE_SRV_NAME;

  explicit RLLMoveIfaceGripperServices();
  ~RLLMoveIfaceGripperServices() override = default;

  /**
   * Enable all available gripper the services.
   */
  void offerGripperServices(ros::NodeHandle* nh);

  // These functions are invoked by the corresponding service calls, they are part of the public interface
  RLLErrorCode moveGripper(const rll_msgs::MoveGripper::Request& req, rll_msgs::MoveGripper::Response* resp);
  RLLErrorCode pickPlace(const rll_msgs::PickPlace::Request& req, rll_msgs::PickPlace::Response* resp);
  RLLErrorCode pickPlaceHere(const rll_msgs::PickPlaceHere::Request& req, rll_msgs::PickPlaceHere::Response* /*resp*/);
  RLLErrorCode validatePickPlace(const rll_msgs::ValidatePickPlace::Request& req,
                                 rll_msgs::ValidatePickPlace::Response* /*resp*/);

  RLLErrorCode resetToHome() override;

  /**
   * Register a new GraspObject whose state will be tracked and can be gripped.
   */
  GraspObject* registerGraspObject(std::unique_ptr<GraspObject> grasp_object_ptr,
                                   const moveit_msgs::CollisionObject& collision_object);

protected:
  Permissions::Index pick_place_permission_;

  RLLErrorCode isGripperOperationConsistent(bool close_gripper);

  /**
   * Evaluate whether a gripper operation specified by the given parameters is valid with respect to the current gripper
   * state.
   */
  RLLErrorCode isGripperAttachmentOperationConsistent(const std::string& object_id, bool close_gripper,
                                                      GraspObject** grasp_object_ptr);

  /**
   * Retrieve a CollisionObject via the PlanningSceneInterface.
   */
  bool getCollisionObjectByID(const std::string& id, moveit_msgs::CollisionObject* object_ptr);

  /**
   * Retrieve an attached CollisionObject via the PlanningSceneInterface.
   */
  bool getAttachedCollisionObjectByID(const std::string& id, moveit_msgs::CollisionObject* object_ptr);

private:
  // internal moveit named targets
  static const std::string GRIPPER_PLANNING_GROUP;
  static const std::string GRIPPER_OPEN_TARGET_NAME;
  static const std::string GRIPPER_CLOSE_TARGET_NAME;
  moveit::planning_interface::MoveGroupInterface gripper_move_group_;
  bool is_gripper_closed_ = false;

  ros::ServiceServer pick_place_service_, validate_pick_place_service_, pick_place_here_service_, move_gripper_service_;

  std::unordered_map<std::string, std::unique_ptr<GraspObject>> grasp_objects_;
  GraspObject* currently_grasped_object_ptr_ = nullptr;

  // Service end-points, whose signature requires references
  // NOLINTNEXTLINE(google-runtime-references)
  bool pickPlaceSrv(rll_msgs::PickPlace::Request& req, rll_msgs::PickPlace::Response& resp);
  // NOLINTNEXTLINE(google-runtime-references)
  bool pickPlaceHereSrv(rll_msgs::PickPlaceHere::Request& req, rll_msgs::PickPlaceHere::Response& resp);
  // NOLINTNEXTLINE(google-runtime-references)
  bool moveGripperSrv(rll_msgs::MoveGripper::Request& req, rll_msgs::MoveGripper::Response& resp);
  // NOLINTNEXTLINE(google-runtime-references)
  bool validatePickPlaceSrv(rll_msgs::ValidatePickPlace::Request& req, rll_msgs::ValidatePickPlace::Response& resp);

  /**
   * Validates whether the requested operation is consistent with the gripper state, all constraints are satisfied and
   * the gripper would not be in collision for the goal pose. Note: this does not guarantee that the actual pickPlace
   * operation would be successful if performed.
   */
  RLLErrorCode validatePickPlaceGripPose(const std::string& object_id, geometry_msgs::Pose grip_pose,
                                         bool close_gripper, GraspObject** grasp_object_ptr);
  RLLErrorCode approachPickPlaceGripPose(const geometry_msgs::Pose& approach_pose,
                                         const geometry_msgs::Pose& grip_pose);
  RLLErrorCode retreatFromPickPlaceGripPose(const geometry_msgs::Pose& retreat_pose);

  /**
   * Closes the gripper.
   */
  RLLErrorCode closeGripper();

  /**
   * Opens the gripper
   */
  RLLErrorCode openGripper();

  /**
   * Close the gripper and attach the GraspObject to the EEF.
   */
  RLLErrorCode closeGripperAndAttach(GraspObject* grasp_object_ptr);

  /**
   * Open the gripper and release the currently attached GrapObject.
   */
  RLLErrorCode openGripperAndDetach();

  /**
   * Remove a CollisionObject from the world and attach it to the EEF.
   */
  bool attachCollisionObject(const moveit_msgs::CollisionObject& collision_object);

  /**
   * Remove a CollisionObject from the EEF and add it back to the world.
   */
  bool detachAttachedCollisionObject(const moveit_msgs::CollisionObject& collision_object);

  /**
   * Find a GraspObject by the specified id of the underlying CollisionObject.
   */
  GraspObject* getGraspObjectByID(const std::string& object_id);

  /**
   * Update the underlying CollisionObject of the given GraspObject.
   */
  bool updateCollisionObject(GraspObject* grasp_object_ptr, bool is_attached);

  void setupPickPlacePermissions();
  void debugGripperState();
  void debugCurrentlyGraspedObject();
};

#endif  // RLL_MOVE_MOVE_IFACE_GRIPPER_H
