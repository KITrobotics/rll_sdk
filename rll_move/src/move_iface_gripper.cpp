/*
 * This file is part of the Robot Learning Lab SDK
 *
 * Copyright (C) 2018-2019 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
 * Copyright (C) 2019-2020 Mark Weinreuter <mark.weinreuter@kit.edu>
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

#include "tf/transform_datatypes.h"
#include <fcl/collision.h>
#include <fcl/intersect.h>
#include <rll_move/move_iface_gripper.h>

const std::string RLLMoveIfaceGripperServices::MOVE_GRIPPER_SRV_NAME = "move_gripper";
const std::string RLLMoveIfaceGripperServices::PICK_PLACE_SRV_NAME = "pick_place";
const std::string RLLMoveIfaceGripperServices::PICK_PLACE_HERE_SRV_NAME = "pick_place_here";
const std::string RLLMoveIfaceGripperServices::VALIDATE_PICK_PLACE_SRV_NAME = "validate_pick_place";
const std::string RLLMoveIfaceGripperServices::GRIPPER_PLANNING_GROUP = "gripper";
const std::string RLLMoveIfaceGripperServices::GRIPPER_OPEN_TARGET_NAME = "gripper_open";
const std::string RLLMoveIfaceGripperServices::GRIPPER_CLOSE_TARGET_NAME = "gripper_close";

RLLMoveIfaceGripperServices::RLLMoveIfaceGripperServices() : gripper_move_group_(GRIPPER_PLANNING_GROUP)
{
  gripper_move_group_.setPlannerId("RRTConnectkConfigDefault");
  gripper_move_group_.setPlanningTime(2.0);
  setupPickPlacePermissions();
}

void RLLMoveIfaceGripperServices::offerGripperServices(ros::NodeHandle* nh)
{
  pick_place_service_ = nh->advertiseService(RLLMoveIfaceGripperServices::PICK_PLACE_SRV_NAME,
                                             &RLLMoveIfaceGripperServices::pickPlaceSrv, this);
  validate_pick_place_service_ = nh->advertiseService(RLLMoveIfaceGripperServices::VALIDATE_PICK_PLACE_SRV_NAME,
                                                      &RLLMoveIfaceGripperServices::validatePickPlaceSrv, this);
  pick_place_here_service_ = nh->advertiseService(RLLMoveIfaceGripperServices::PICK_PLACE_HERE_SRV_NAME,
                                                  &RLLMoveIfaceGripperServices::pickPlaceHereSrv, this);

  move_gripper_service_ = nh->advertiseService(RLLMoveIfaceGripperServices::MOVE_GRIPPER_SRV_NAME,
                                               &RLLMoveIfaceGripperServices::moveGripperSrv, this);
}

RLLErrorCode RLLMoveIfaceGripperServices::resetToHome()
{
  RLLErrorCode error_code = RLLMoveIfaceServices::resetToHome();
  if (error_code.failed())
  {
    return error_code;
  }

  // only try to open the gripper if it is actually closed
  if (!is_gripper_closed_)
  {
    return RLLErrorCode::SUCCESS;
  }

  return openGripper();
}

void RLLMoveIfaceGripperServices::setupPickPlacePermissions()
{
  pick_place_permission_ = permissions_.registerPermission("pick_place", false);

  // the pick_and_place service requires an additional permission
  permissions_.setRequiredPermissionsFor(RLLMoveIfaceGripperServices::PICK_PLACE_SRV_NAME, pick_place_permission_,
                                         true);
  permissions_.setRequiredPermissionsFor(RLLMoveIfaceGripperServices::PICK_PLACE_HERE_SRV_NAME, pick_place_permission_,
                                         true);
  permissions_.setRequiredPermissionsFor(RLLMoveIfaceGripperServices::MOVE_GRIPPER_SRV_NAME, pick_place_permission_,
                                         true);
}

RLLErrorCode RLLMoveIfaceGripperServices::isGripperOperationConsistent(bool close_gripper)
{
  if (is_gripper_closed_)
  {
    if (close_gripper)
    {
      ROS_WARN("Cannot close the gripper, the gripper is already closed.");
      return RLLErrorCode::INVALID_INPUT;
    }
  }
  else
  {
    if (!close_gripper)
    {
      ROS_WARN("Cannot open gripper, the gripper is already open.");
      return RLLErrorCode::INVALID_INPUT;
    }
  }
  return RLLErrorCode::SUCCESS;
}

RLLErrorCode RLLMoveIfaceGripperServices::isGripperAttachmentOperationConsistent(const std::string& object_id,
                                                                                 bool close_gripper,
                                                                                 GraspObject** grasp_object_ptr)
{
  RLLErrorCode error_code = isGripperOperationConsistent(close_gripper);
  if (!error_code.succeeded())
  {
    return error_code;
  }

  // are we trying to detach an object that isn't attached?
  if (!close_gripper)
  {
    if (currently_grasped_object_ptr_ != nullptr)
    {
      const std::string ID = currently_grasped_object_ptr_->getID();
      if (ID != object_id)
      {
        ROS_WARN("Object '%s' is currently grasped, cannot open gripper and release different object: '%s'", ID.c_str(),
                 object_id.c_str());
        return RLLErrorCode::GRIPPER_OPERATION_FAILED;
      }
    }
    *grasp_object_ptr = currently_grasped_object_ptr_;
  }
  else
  {
    *grasp_object_ptr = getGraspObjectByID(object_id);
    if (*grasp_object_ptr == nullptr)
    {
      ROS_ERROR("No such grasp object: %s", object_id.c_str());
      return RLLErrorCode::INVALID_INPUT;
    }
  }
  return RLLErrorCode::SUCCESS;
}

GraspObject* RLLMoveIfaceGripperServices::registerGraspObject(std::unique_ptr<GraspObject> grasp_object_ptr,
                                                              const moveit_msgs::CollisionObject& collision_object)
{
  bool success = planning_scene_interface_.applyCollisionObject(collision_object);
  if (!success)
  {
    ROS_ERROR("Failed to add collision object!");
    return nullptr;
  }
  grasp_object_ptr->updateDataFromCollisionObject(collision_object);
  const std::string ID = collision_object.id;

  grasp_objects_[ID] = std::move(grasp_object_ptr);
  ROS_INFO_STREAM("Registered new collision object with ID: " << ID);

  return grasp_objects_[ID].get();  // TODO(mark): is it ok to return the pointer here?
}

// TODO(mark): is it smart to return the raw pointer of our smart pointer?
GraspObject* RLLMoveIfaceGripperServices::getGraspObjectByID(const std::string& object_id)
{
  auto iter = grasp_objects_.find(object_id);
  if (iter != grasp_objects_.end())
  {
    return iter->second.get();
  }
  return nullptr;
}

bool RLLMoveIfaceGripperServices::getCollisionObjectByID(const std::string& id,
                                                         moveit_msgs::CollisionObject* object_ptr)
{
  std::map<std::string, moveit_msgs::CollisionObject> objects =
      planning_scene_interface_.getObjects(std::vector<std::string>{ id });

  const auto ITER = objects.find(id);
  if (ITER != objects.end())
  {
    *object_ptr = ITER->second;
    return true;
  }

  return false;
}

bool RLLMoveIfaceGripperServices::getAttachedCollisionObjectByID(const std::string& id,
                                                                 moveit_msgs::CollisionObject* object_ptr)
{
  std::map<std::string, moveit_msgs::AttachedCollisionObject> objects =
      planning_scene_interface_.getAttachedObjects(std::vector<std::string>{ id });

  const auto ITER = objects.find(id);
  if (ITER != objects.end())
  {
    *object_ptr = ITER->second.object;
    return true;
  }

  return false;
}

bool RLLMoveIfaceGripperServices::updateCollisionObject(GraspObject* grasp_object_ptr, bool is_attached)
{
  moveit_msgs::CollisionObject tmp_collision_obj;
  bool found_object = false;
  if (is_attached)
  {
    found_object = getAttachedCollisionObjectByID(grasp_object_ptr->getID(), &tmp_collision_obj);
  }
  else
  {
    found_object = getCollisionObjectByID(grasp_object_ptr->getID(), &tmp_collision_obj);
  }

  if (!found_object)
  {
    ROS_ERROR("Failed to update collision object!");
    return false;
  }
  grasp_object_ptr->updateDataFromCollisionObject(tmp_collision_obj);
  return true;
}

bool RLLMoveIfaceGripperServices::moveGripperSrv(rll_msgs::MoveGripper::Request& req,
                                                 rll_msgs::MoveGripper::Response& resp)
{
  // TODO(mark): remove this service? Is it needed and safe?
  return controlledMovementExecution(req, &resp, MOVE_GRIPPER_SRV_NAME, &RLLMoveIfaceGripperServices::moveGripper);
}

bool RLLMoveIfaceGripperServices::pickPlaceSrv(rll_msgs::PickPlace::Request& req, rll_msgs::PickPlace::Response& resp)
{
  return controlledMovementExecution(req, &resp, PICK_PLACE_SRV_NAME, &RLLMoveIfaceGripperServices::pickPlace);
}

bool RLLMoveIfaceGripperServices::pickPlaceHereSrv(rll_msgs::PickPlaceHere::Request& req,
                                                   rll_msgs::PickPlaceHere::Response& resp)
{
  return controlledMovementExecution(req, &resp, PICK_PLACE_HERE_SRV_NAME, &RLLMoveIfaceGripperServices::pickPlaceHere);
}

bool RLLMoveIfaceGripperServices::validatePickPlaceSrv(rll_msgs::ValidatePickPlace::Request& req,
                                                       rll_msgs::ValidatePickPlace::Response& resp)
{
  return controlledMovementExecution(req, &resp, PICK_PLACE_HERE_SRV_NAME,
                                     &RLLMoveIfaceGripperServices::validatePickPlace);
}

RLLErrorCode RLLMoveIfaceGripperServices::moveGripper(const rll_msgs::MoveGripper::Request& req,
                                                      rll_msgs::MoveGripper::Response* /*resp*/)
{
  bool close_gripper = req.gripper_close != 0u;
  if (!close_gripper && currently_grasped_object_ptr_ != nullptr)
  {
    ROS_ERROR("Cannot open the gripper if an object is attached, use pick_place instead.");
    return RLLErrorCode::INVALID_INPUT;
  }

  ros::Duration(0.25).sleep();  // without the delay the position at which the grasp object is attached is wrong!
  getCurrentRobotState(true).update();  // TODO(mark): does this help? is it unnecessary try to update the scene?

  if (close_gripper)
  {
    return closeGripper();
  }
  return openGripper();
}

RLLErrorCode RLLMoveIfaceGripperServices::pickPlace(const rll_msgs::PickPlace::Request& req,
                                                    rll_msgs::PickPlace::Response* /*resp*/)
{
  // TODO(mark): validate the poses we are given are valid! E.g. orientation is normalized, this should be done for
  // all movement services

  RLLErrorCode error_code;
  // TODO(mark): in what cases should the pickPlace operation result in an internal error? E.g. failing to open/close
  // the gripper?
  bool close_gripper = req.gripper_close != 0u;

  GraspObject* grasp_object_ptr = nullptr;
  error_code = validatePickPlaceGripPose(req.grasp_object, req.pose_grip, close_gripper, &grasp_object_ptr);
  if (error_code.failed())
  {
    return error_code;
  }
  ROS_ASSERT(grasp_object_ptr != nullptr);

  error_code = approachPickPlaceGripPose(req.pose_approach, req.pose_grip);
  if (error_code.failed())
  {
    return error_code;
  }

  // actually perform the gripping operation and store the error separately
  ros::Duration(0.25).sleep();  // without the delay the position at which the grasp object is attached is wrong!
  getCurrentRobotState(true).update();  // TODO(mark): does this help? is it unnecessary try to update the scene?

  RLLErrorCode gripper_error_code = close_gripper ? closeGripperAndAttach(grasp_object_ptr) : openGripperAndDetach();

  if (gripper_error_code.isCriticalFailure())
  {  // allow to move to the retreat pose on non critical errors
    ROS_ERROR("pickPlace: Gripper motion failed critically, aborting pick place.");
    return gripper_error_code;
  }

  error_code = retreatFromPickPlaceGripPose(req.pose_retreat);

  // in case grasping failed (not critically), and the retreat was successful still return the grasping error
  if (gripper_error_code.failed() and error_code.succeeded())
  {
    return gripper_error_code;
  }

  return error_code;
}

RLLErrorCode RLLMoveIfaceGripperServices::pickPlaceHere(const rll_msgs::PickPlaceHere::Request& req,
                                                        rll_msgs::PickPlaceHere::Response* /*resp*/)
{
  // use the current pose as the approach/retreat pose
  geometry_msgs::Pose current = getCurrentPoseFromPlanningScene();
  rll_msgs::PickPlace::Response pp_response;  // is unused in pickPlace
  rll_msgs::PickPlace::Request pp_request;
  pp_request.pose_approach = current;
  pp_request.pose_retreat = current;
  pp_request.pose_grip = req.pose_grip;
  pp_request.grasp_object = req.grasp_object;
  pp_request.gripper_close = req.gripper_close;

  return pickPlace(pp_request, &pp_response);
}

RLLErrorCode RLLMoveIfaceGripperServices::validatePickPlace(const rll_msgs::ValidatePickPlace::Request& req,
                                                            rll_msgs::ValidatePickPlace::Response* /*resp*/)
{
  GraspObject* grasp_object_ptr = nullptr;
  bool close_gripper = req.gripper_close != 0u;
  return validatePickPlaceGripPose(req.grasp_object, req.pose_grip, close_gripper, &grasp_object_ptr);
}

RLLErrorCode RLLMoveIfaceGripperServices::validatePickPlaceGripPose(const std::string& object_id,
                                                                    const geometry_msgs::Pose grip_pose,
                                                                    bool close_gripper, GraspObject** grasp_object_ptr)
{
  RLLErrorCode error_code = isGripperAttachmentOperationConsistent(object_id, close_gripper, grasp_object_ptr);
  if (error_code.failed())
  {
    return error_code;
  }

  ROS_ASSERT(grasp_object_ptr != nullptr);
  if (close_gripper)
  {
    geometry_msgs::Pose target_object_pose = (*grasp_object_ptr)->getObjectPoseForEEFPose(grip_pose);
    bool target_in_bbox = (*grasp_object_ptr)->pointInCollisionGeometry(target_object_pose, grip_pose.position);
    if (!target_in_bbox)
    {
      ROS_INFO(
          "The grasp pose is not within the collision BBox of the GraspObject -> Impossible to pick up the object!");
      return RLLErrorCode::GRIPPER_CONSTRAINT_FAILED;
    }

    if (!(*grasp_object_ptr)->canBeGrasped(grip_pose))
    {
      ROS_INFO("Not all pick constraints are satisfied!");
      return RLLErrorCode::GRIPPER_CONSTRAINT_FAILED;
    }
  }

  if (!close_gripper)
  {
    if (!(*grasp_object_ptr)->canBeReleased(grip_pose))
    {
      ROS_INFO("Not all place constraints are satisfied!");
      return RLLErrorCode::GRIPPER_CONSTRAINT_FAILED;
    }
  }

  // TODO(mark): the IK does not respect collision checks yet!
  error_code = poseGoalInCollision(grip_pose);
  if (error_code.failed())
  {
    ROS_INFO("PickPlace failed, because grip pose can not be reached, it might be in a colliding state.");
  }

  return error_code;
}

RLLErrorCode RLLMoveIfaceGripperServices::approachPickPlaceGripPose(const geometry_msgs::Pose& approach_pose,
                                                                    const geometry_msgs::Pose& grip_pose)
{
  if (!tooCloseForLinearMovement(approach_pose))
  {
    ROS_INFO_POS("[pickPlace] Moving to approach position", approach_pose.position);
    // TOOD(mark): linear only if no object is currently grasped?
    RLLErrorCode error_code = moveToGoalLinear(approach_pose);

    if (error_code.failed())
    {
      ROS_WARN("pickPlace: Failed to move to approach position");
      return error_code;
    }

    ROS_INFO("pickPlace: Reached approach position");
  }
  else
  {
    ROS_INFO_POS("Already at approach position", approach_pose.position);
  }

  ROS_INFO_POSE("Moving to grip pose", grip_pose);
  RLLErrorCode error_code = moveToGoalLinear(grip_pose);
  if (error_code.failed())
  {
    ROS_WARN("pickPlace: Moving to grip position failed");
    return error_code;
  }

  ROS_INFO("pickPlace: moved to grip position");
  return RLLErrorCode::SUCCESS;
}

RLLErrorCode RLLMoveIfaceGripperServices::retreatFromPickPlaceGripPose(const geometry_msgs::Pose& retreat_pose)
{
  ROS_INFO_POS("[pickPlace] Retreating to position", retreat_pose.position);

  RLLErrorCode error_code = moveToGoalLinear(retreat_pose);
  if (error_code.failed())
  {
    ROS_WARN("pickPlace: Retreating from grip position failed");
  }
  else
  {
    ROS_INFO("pickPlace: moved to retreat position");
  }
  return error_code;
}

RLLErrorCode RLLMoveIfaceGripperServices::closeGripper()
{
  if (!manipCurrentStateAvailable())
  {
    return RLLErrorCode::MANIPULATOR_NOT_AVAILABLE;
  }
  ROS_INFO("Closing the gripper");
  RLLErrorCode error_code = isGripperOperationConsistent(true);
  if (!error_code.succeeded())
  {
    return error_code;
  }

  gripper_move_group_.setStartStateToCurrentState();
  gripper_move_group_.setNamedTarget(GRIPPER_CLOSE_TARGET_NAME);

  error_code = runPTPTrajectory(&gripper_move_group_, true);
  if (error_code.failed())
  {
    ROS_FATAL("Failed to close the gripper");
    return error_code;
  }

  ROS_INFO("Gripper is closed");
  is_gripper_closed_ = true;

  return error_code;
}

RLLErrorCode RLLMoveIfaceGripperServices::openGripper()
{
  if (!manipCurrentStateAvailable())
  {
    return RLLErrorCode::MANIPULATOR_NOT_AVAILABLE;
  }
  ROS_INFO("Opening the gripper");
  RLLErrorCode error_code = isGripperOperationConsistent(false);
  if (!error_code.succeeded())
  {
    return error_code;
  }

  gripper_move_group_.setStartStateToCurrentState();
  gripper_move_group_.setNamedTarget(GRIPPER_OPEN_TARGET_NAME);

  error_code = runPTPTrajectory(&gripper_move_group_, true);
  if (error_code.failed())
  {
    ROS_FATAL("Failed to open the gripper");
    // override the error code, failing to open the gripper is critical
    return RLLErrorCode::GRIPPER_OPERATION_FAILED;
  }

  ROS_INFO("Gripper is open");
  is_gripper_closed_ = false;
  return error_code;
}

bool RLLMoveIfaceGripperServices::attachCollisionObject(const moveit_msgs::CollisionObject& collision_object)
{
  moveit_msgs::CollisionObject to_remove;
  to_remove.id = collision_object.id;
  to_remove.operation = moveit_msgs::CollisionObject::REMOVE;  // Check whether it is currently attached?

  // 1. remove the (unattached) object from the scene
  bool result = planning_scene_interface_.applyCollisionObject(to_remove);
  if (!result)
  {
    ROS_INFO("Failed to remove CollisionObject '%s'", to_remove.id.c_str());
    return false;
  }

  // 2. attach the collision_object to the EEF
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = manip_move_group_.getEndEffectorLink();
  attached_object.object = collision_object;
  attached_object.object.operation = moveit_msgs::CollisionObject::ADD;
  // these links will be ignored for collisions with the grasp collision_object
  attached_object.touch_links = std::vector<std::string>{ getNamespace() + "_" + getEEFType() + "_finger_left",
                                                          getNamespace() + "_" + getEEFType() + "_finger_right" };

  result = planning_scene_interface_.applyAttachedCollisionObject(attached_object);
  if (!result)
  {
    ROS_ERROR("Failed to add AttachCollisionObject: %s", attached_object.object.id.c_str());
    // TODO(mark): what now?
    return false;
  }

  // ROS_INFO_STREAM("AttachCollisionObject: \n" << attached_object);
  // TODO(mark): figure out if this is really needed
  // occasionally, there seems to be a race condition with subsequent planning requests
  ros::Duration(0.25).sleep();

  return result;
}

bool RLLMoveIfaceGripperServices::detachAttachedCollisionObject(const moveit_msgs::CollisionObject& collision_object)
{
  ROS_INFO("Detaching grasp object '%s'", collision_object.id.c_str());

  // 1. remove the previous attachment, set only the mandatory fields (no need to publish the shape on removal)
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = manip_move_group_.getEndEffectorLink();
  attached_object.object.id = collision_object.id;
  attached_object.object.operation = moveit_msgs::CollisionObject::REMOVE;
  bool result = planning_scene_interface_.applyAttachedCollisionObject(attached_object);
  if (!result)
  {
    ROS_ERROR("Failed to detach AttachCollisionObject object: %s", attached_object.object.id.c_str());
    return false;
  }

  // 2. add the object back to the scene
  // TODO(mark): disable collision between world and object? -> Ensure there is  small gap, e.g. 1mm?
  // TODO(mark): validate the collision object is in the right location after detach
  attached_object.object = collision_object;
  attached_object.object.operation = moveit_msgs::CollisionObject::ADD;
  result = planning_scene_interface_.applyCollisionObject(attached_object.object);

  if (!result)
  {
    ROS_ERROR("Failed to add the CollisionObject to the scene after detaching it: %s", collision_object.id.c_str());
    return false;
  }

  // ROS_INFO_STREAM("Detached object and added it back to the scene: \n" << collision_object);
  // TODO(mark): figure out if this is really needed
  // occasionally, there seems to be a race condition with subsequent planning requests
  ros::Duration(0.25).sleep();
  return true;
}

RLLErrorCode RLLMoveIfaceGripperServices::openGripperAndDetach()
{
  ROS_ASSERT(currently_grasped_object_ptr_ != nullptr);

  // make sure the grasp object has an up to date collision object
  if (!updateCollisionObject(currently_grasped_object_ptr_, true))
  {
    // TODO(mark): what now?
    return RLLErrorCode::GRIPPER_OPERATION_FAILED;
  }

  geometry_msgs::Pose eef_pose = getCurrentPoseFromPlanningScene();
  bool can_be_placed_here = currently_grasped_object_ptr_->canBeReleased(eef_pose);
  if (!can_be_placed_here)
  {
    return RLLErrorCode::GRIPPER_CONSTRAINT_FAILED;
  }

  RLLErrorCode error_code = openGripper();
  if (error_code.failed())
  {
    return error_code;
  }

  // detach the moveit collision object after successfully opening the gripper
  bool result = detachAttachedCollisionObject(currently_grasped_object_ptr_->getCollisionObject());
  if (!result)
  {
    ROS_FATAL("Something went wrong detaching the collision object!");
    // TODO(mark): what now?
    return RLLErrorCode::GRIPPER_OPERATION_FAILED;
  }

  // update the collision object again
  if (!updateCollisionObject(currently_grasped_object_ptr_, false))
  {
    // TODO(mark): what now?
    return RLLErrorCode::GRIPPER_OPERATION_FAILED;
  }

  const geometry_msgs::Pose OBJECT_POSE = currently_grasped_object_ptr_->getCenterPose();
  currently_grasped_object_ptr_->hasBeenReleased(eef_pose, OBJECT_POSE);
  currently_grasped_object_ptr_ = nullptr;

  return RLLErrorCode::SUCCESS;
}

RLLErrorCode RLLMoveIfaceGripperServices::closeGripperAndAttach(GraspObject* grasp_object_ptr)
{
  ROS_ASSERT(grasp_object_ptr != nullptr);

  // make sure the grasp object has an up to date collision object
  if (!updateCollisionObject(grasp_object_ptr, false))
  {
    return RLLErrorCode::GRIPPER_OPERATION_FAILED;
  }

  geometry_msgs::Pose eef_target_pose = getCurrentPoseFromPlanningScene();
  bool is_possible = grasp_object_ptr->canBeGrasped(eef_target_pose);
  if (!is_possible)
  {
    return RLLErrorCode::GRIPPER_CONSTRAINT_FAILED;
  }

  // attach the object first, which will prevent collision checks between the gripper fingers and the object
  bool attachment_result = attachCollisionObject(grasp_object_ptr->getCollisionObject());
  if (!attachment_result)
  {
    ROS_FATAL("Attachment failed");
    // TODO(mark): now what?
    return RLLErrorCode::INVALID_GRIPPER_OPERATION;
  }

  RLLErrorCode error_code = closeGripper();

  if (error_code.failed())
  {
    // undo the attachment since gripping failed
    detachAttachedCollisionObject(grasp_object_ptr->getCollisionObject());
    return error_code;
  }

  geometry_msgs::Pose unattached_object_pose = grasp_object_ptr->getCenterPose();

  // update the now attached object
  if (!updateCollisionObject(grasp_object_ptr, true))
  {
    return RLLErrorCode::GRIPPER_OPERATION_FAILED;
  }

  grasp_object_ptr->hasBeenGrasped(eef_target_pose, unattached_object_pose);
  currently_grasped_object_ptr_ = grasp_object_ptr;

  return RLLErrorCode::SUCCESS;
}

void RLLMoveIfaceGripperServices::debugGripperState()
{
  ROS_INFO("Gripper status: closed=%d\n", is_gripper_closed_);
  if (currently_grasped_object_ptr_ != nullptr)
  {
    ROS_INFO("-> attached=%s\n", currently_grasped_object_ptr_->getID().c_str());
  }
}

void RLLMoveIfaceGripperServices::debugCurrentlyGraspedObject()
{
  if (currently_grasped_object_ptr_ != nullptr)
  {
    updateCollisionObject(currently_grasped_object_ptr_, true);

    ROS_INFO("\nCurrently grasped object:\n");
    ROS_INFO_STREAM("EEF pose: " << getCurrentPoseFromPlanningScene());
    ROS_INFO_STREAM("Attached pose: " << currently_grasped_object_ptr_->getCenterPose());
    geometry_msgs::Pose world_obj_pose =
        currently_grasped_object_ptr_->getObjectPoseForEEFPose(getCurrentPoseFromPlanningScene());
    ROS_INFO_STREAM("Object world pose: " << world_obj_pose);
  }
  else
  {
    ROS_INFO("No object currently attached.");
  }
}
