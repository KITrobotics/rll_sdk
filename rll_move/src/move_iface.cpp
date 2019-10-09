/*
 * This file is part of the Robot Learning Lab SDK
 *
 * Copyright (C) 2018 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
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

#include <rll_move/move_iface.h>
#include <tf2_ros/transform_listener.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <eigen_conversions/eigen_msg.h>

const std::string RLLMoveIface::MANIP_PLANNING_GROUP = "manipulator";
const std::string RLLMoveIface::GRIPPER_PLANNING_GROUP = "gripper";

const std::string RLLMoveIface::IDLE_JOB_SRV_NAME = "job_idle";
const std::string RLLMoveIface::RUN_JOB_SRV_NAME = "job_env";

const std::string RLLMoveIface::ROBOT_READY_SRV_NAME = "robot_ready";
const std::string RLLMoveIface::MOVE_PTP_SRV_NAME = "move_ptp";
const std::string RLLMoveIface::MOVE_PTP_ELB_SRV_NAME = "move_ptp_elb";
const std::string RLLMoveIface::MOVE_LIN_SRV_NAME = "move_lin";
const std::string RLLMoveIface::MOVE_LIN_ELB_SRV_NAME = "move_lin_elb";
const std::string RLLMoveIface::MOVE_JOINTS_SRV_NAME = "move_joints";
const std::string RLLMoveIface::MOVE_RANDOM_SRV_NAME = "move_random";
const std::string RLLMoveIface::PICK_PLACE_SRV_NAME = "pick_place";
const std::string RLLMoveIface::GET_POSE_SRV_NAME = "get_current_pose";
const std::string RLLMoveIface::GET_JOINT_VALUES_SRV_NAME = "get_current_joint_values";

const std::string RLLMoveIface::HOME_TARGET_NAME = "home_bow";
const std::string RLLMoveIface::GRIPPER_OPEN_TARGET_NAME = "gripper_open";
const std::string RLLMoveIface::GRIPPER_CLOSE_TARGET_NAME = "gripper_close";

RLLMoveIface::RLLMoveIface() : manip_move_group_(MANIP_PLANNING_GROUP), gripper_move_group_(GRIPPER_PLANNING_GROUP)
{
  ns_ = ros::this_node::getNamespace();
// remove the slashes at the beginning
#if ROS_VERSION_MINIMUM(1, 14, 3)  // Melodic
  ns_.erase(0, 1);
#else  // Kinetic and older
  ns_.erase(0, 2);
#endif
  ROS_INFO("starting in ns %s", ns_.c_str());

  node_name_ = ros::this_node::getName();
  ros::param::get(node_name_ + "/no_gripper", no_gripper_attached_);
  if (no_gripper_attached_)
  {
    ROS_INFO("configured to not use a gripper");
  }

  manip_move_group_.setPlannerId("RRTConnectkConfigDefault");
  manip_move_group_.setPlanningTime(2.0);
  manip_move_group_.setPoseReferenceFrame("world");
  gripper_move_group_.setPlannerId("RRTConnectkConfigDefault");
  gripper_move_group_.setPlanningTime(2.0);

  manip_model_ = manip_move_group_.getRobotModel();
  manip_joint_model_group_ = manip_model_->getJointModelGroup(manip_move_group_.getName());

  std::string ee_link = ns_ + "_gripper_link_ee";
  manip_move_group_.setEndEffectorLink(ee_link);

  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

  planning_scene_monitor_->requestPlanningSceneState("get_planning_scene");
  planning_scene_ = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_);
  acm_ = planning_scene_->getAllowedCollisionMatrix();

  // set the secret (if any) that is required to invoke actions
  std::string secret;
  ros::param::get(node_name_ + "/authentication_secret", secret);
  authentication_.setSecret(secret);

  setupPermissions();

  // startup checks, shutdown the node if something is wrong
  if (!isCollisionLinkAvailable() || !initConstTransforms())
  {
    ROS_FATAL("Startup checks failed, shutting the node down!");
    ros::shutdown();
  }
}

void RLLMoveIface::setupPermissions()
{
  only_during_job_run_permission_ = permissions_.registerPermission("only_during_job_run", false);
  move_permission_ = permissions_.registerPermission("allowed_to_move", false);
  pick_place_permission_ = permissions_.registerPermission("pick_place", false);

  // by default every service requires the move permission unless explicitly changed
  auto default_permissions = only_during_job_run_permission_ | move_permission_;
  permissions_.setDefaultRequiredPermissions(default_permissions);

  // the pick_and_place service requires an additional permission
  permissions_.setRequiredPermissionsFor(RLLMoveIface::PICK_PLACE_SRV_NAME,
                                         pick_place_permission_ | default_permissions);
  // robot ready service check is always allowed
  permissions_.setRequiredPermissionsFor(RLLMoveIface::ROBOT_READY_SRV_NAME, Permissions::NO_PERMISSION_REQUIRED);
}

bool RLLMoveIface::beforeActionExecution(RLLMoveIfaceState state, const std::string& secret,
                                         rll_msgs::JobEnvResult* result)
{
  // before an action is executed we need to ensure that the caller is authorized to invoke an action
  if (!authentication_.authenticate(secret))
  {
    ROS_FATAL("Authentication is required to execute an action!");
    result->job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
    return false;
  }

  // try to enter the corresponding state
  if (!iface_state_.enterState(state))
  {
    result->job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
    return false;
  }

  return true;
}

bool RLLMoveIface::afterActionExecution(rll_msgs::JobEnvResult* result)
{
  if (!iface_state_.leaveState())
  {
    result->job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
    return false;
  }

  return true;
}

void RLLMoveIface::runJobAction(const rll_msgs::JobEnvGoalConstPtr& goal, JobServer* as)
{
  rll_msgs::JobEnvResult result;

  if (!beforeActionExecution(RLLMoveIfaceState::RUNNING_JOB, goal->authentication_secret, &result))
  {
    // don't attempt to perform an action if we are in the INTERNAL_ERROR state
    as->setSucceeded(result);
    return;
  }

  // run the actual job processing and set the result accordingly
  // set the general movement permission for the duration of the job execution
  permissions_.storeCurrentPermissions();
  permissions_.updateCurrentPermissions(move_permission_ | only_during_job_run_permission_ | pick_place_permission_,
                                        true);
  runJob(goal, result);
  permissions_.restorePreviousPermissions();

  // It is possible that a service call might still be in execution
  // therefore wait for the service call to end before completing the runJob action
  if (iface_state_.setCurrentServiceCallResult(RLLErrorCode::JOB_EXECUTION_TIMED_OUT))
  {
    while (iface_state_.isServiceCallInExecution())
    {
      ros::Duration(.01).sleep();
    }
  }

  // sanity check: if a job fails with an internal error than further operations should have been aborted
  if (result.job.status == rll_msgs::JobStatus::INTERNAL_ERROR && !iface_state_.isInInternalErrorState())
  {
    ROS_FATAL("Job resulted in an INTERNAL_ERROR, but the state is not set accordingly. This should NOT happen!");
    abortDueToCriticalFailure();
  }

  afterActionExecution(&result);
  as->setSucceeded(result);
}

void RLLMoveIface::idleAction(const rll_msgs::JobEnvGoalConstPtr& goal, JobServer* as)
{
  rll_msgs::JobEnvResult result;

  if (!beforeActionExecution(RLLMoveIfaceState::IDLING, goal->authentication_secret, &result))
  {
    // don't attempt to perform the action we are in the INTERNAL_ERROR state
    as->setSucceeded(result);
    return;
  }

  ROS_INFO("got idle request");
  RLLErrorCode error_code = idle();

  if (error_code.succeeded())
  {
    ROS_INFO("Idle succeeded!");
    result.job.status = rll_msgs::JobStatus::SUCCESS;
  }
  else
  {
    ROS_FATAL("Idle resulted in a %s failure!", error_code.isCriticalFailure() ? "critical" : "recoverable");
    // idle should not fail, so its better to abort further operations
    abortDueToCriticalFailure();
    result.job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
  }

  afterActionExecution(&result);
  as->setSucceeded(result);
}

RLLErrorCode RLLMoveIface::idle()
{
  RLLErrorCode error_code = resetToHome();

  if (error_code.failed())
  {
    ROS_ERROR("resetToHome failed with error_code: %s", error_code.message());
    // escalate the error to ensure it is treated as a critical error
    // it might only be a planning failure but resetToHome may never fail
    return RLLErrorCode::RESET_TO_HOME_FAILED;
  }

  return RLLErrorCode::SUCCESS;
}

bool RLLMoveIface::robotReadySrv(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& resp)
{
  RLLErrorCode error_code = beforeMovementServiceCall(RLLMoveIface::ROBOT_READY_SRV_NAME);
  if (error_code.succeeded())
  {
    error_code = resetToHome();
  }
  resp.success = error_code.succeeded();  // NOLINT

  afterMovementServiceCall(RLLMoveIface::ROBOT_READY_SRV_NAME, error_code);
  return true;
}

bool RLLMoveIface::isCollisionLinkAvailable()
{
  std::string collision_link;
  bool success = ros::param::get(node_name_ + "/collision_link", collision_link);
  if (!success)
  {
    ROS_FATAL("No 'collision_link' param set. Please specify a collision_link param "
              "to verify that the collision model is loaded.");
    return false;
  }

  tf2_ros::Buffer tf_buffer;
  // required to allow specifying a timeout in lookupTransform
  tf2_ros::TransformListener listener(tf_buffer);

  // if the workcell is loaded correctly the collision_link should be available
  success = tf_buffer.canTransform("world", collision_link, ros::Time(0), ros::Duration(5));

  if (!success)
  {
    ROS_FATAL("Failed to look up the collision link '%s'. Did you launch the correct file?", collision_link.c_str());
    return false;
  }

  ROS_DEBUG("collision link '%s' lookup succeeded", collision_link.c_str());
  return true;
}

bool RLLMoveIface::manipCurrentStateAvailable()
{
  // Sometimes, the current state cannot be retrieved and a NULL pointer exception is thrown
  // somewhere. Check here if the current state can be retrieved. Other methods can use this
  // function to abort further MoveIt commands and to avoid sigterms.

  robot_state::RobotStatePtr current_state = manip_move_group_.getCurrentState();
  if (current_state == nullptr)
  {
    ROS_FATAL("Current robot state cannot be retrieved.");
    return false;
  }

  return true;
}

void RLLMoveIface::abortDueToCriticalFailure()
{
  iface_state_.enterErrorState();
}

void RLLMoveIface::handleFailureSeverity(const RLLErrorCode& error_code)
{
  // check if a error condition matches, if not assume critical failure

  if (error_code.isInvalidInput())
  {
    ROS_WARN("A failure due to invalid input occurred. error: %s", error_code.message());
  }
  else if (error_code.isRecoverableFailure())
  {
    ROS_WARN("A recoverable failure occurred, further operations are still possible. error: %s", error_code.message());
  }
  else
  {
    ROS_FATAL("A critical failure occurred! All further operations will be cancelled. error: %s", error_code.message());

    abortDueToCriticalFailure();
  }
}

RLLErrorCode RLLMoveIface::beforeNonMovementServiceCall(const std::string& srv_name)
{
  ROS_DEBUG("service '%s' requested", srv_name.c_str());

  bool only_during_job_run = permissions_.isPermissionRequiredFor(srv_name, only_during_job_run_permission_);
  RLLErrorCode error_code = iface_state_.beginServiceCall(srv_name, only_during_job_run);
  if (error_code.failed())
  {
    return error_code;
  }

  // check if this service call is permitted
  if (!permissions_.areAllRequiredPermissionsSetFor(srv_name))
  {
    return RLLErrorCode::INSUFFICIENT_PERMISSION;
  }

  return RLLErrorCode::SUCCESS;
}

RLLErrorCode RLLMoveIface::afterNonMovementServiceCall(const std::string& srv_name, RLLErrorCode previous_error_code)
{
  bool only_during_job_run = permissions_.isPermissionRequiredFor(srv_name, only_during_job_run_permission_);
  RLLErrorCode error_code = iface_state_.endServiceCall(srv_name, only_during_job_run);
  ROS_DEBUG("service '%s' ended", srv_name.c_str());

  // a previous error code is probably more specific and takes precedence
  return previous_error_code.failed() ? previous_error_code : error_code;
}

RLLErrorCode RLLMoveIface::beforeMovementServiceCall(const std::string& srv_name)
{
  RLLErrorCode error_code = beforeNonMovementServiceCall(srv_name);
  if (error_code.failed())
  {
    return error_code;
  }

  if (!manipCurrentStateAvailable())
  {
    return RLLErrorCode::MANIPULATOR_NOT_AVAILABLE;
  }

  return RLLErrorCode::SUCCESS;
}

RLLErrorCode RLLMoveIface::afterMovementServiceCall(const std::string& srv_name,
                                                    const RLLErrorCode& previous_error_code)
{
  // pass a possible previous error_code, it will be returned if it is more specific
  RLLErrorCode error_code = afterNonMovementServiceCall(srv_name, previous_error_code);

  if (error_code.failed())
  {
    ROS_WARN("'%s' service call failed!", srv_name.c_str());
    handleFailureSeverity(error_code);
  }

  return error_code;
}

bool RLLMoveIface::moveRandomSrv(rll_msgs::MoveRandom::Request& req, rll_msgs::MoveRandom::Response& resp)
{
  return controlledMovementExecution(req, resp, MOVE_RANDOM_SRV_NAME, &RLLMoveIface::moveRandom);
}

RLLErrorCode RLLMoveIface::moveRandom(rll_msgs::MoveRandom::Request& /*req*/, rll_msgs::MoveRandom::Response& resp)
{
  bool success = false;
  int retry_counter = 0;
  geometry_msgs::Pose random_pose;
  geometry_msgs::Pose start = manip_move_group_.getCurrentPose().pose;

  while (!success && retry_counter < 30)
  {
    retry_counter++;

    random_pose = manip_move_group_.getRandomPose().pose;
    if (poseGoalTooClose(start, random_pose))
    {
      success = false;
      ROS_INFO("last random pose too close to start pose, retrying...");
      continue;
    }
    RLLErrorCode error_code = poseGoalInCollision(random_pose);
    if (error_code.failed())
    {
      success = false;
      ROS_INFO("last random pose is in collision, retrying...");
      continue;
    }

    success = manip_move_group_.setPoseTarget(random_pose);
    if (!success)
    {
      ROS_INFO("last random pose could not be set as target, retrying...");
      continue;
    }

    error_code = runPTPTrajectory(manip_move_group_);
    // make sure nothing major went wrong. only repeat in case of non critical errors
    if (error_code.isCriticalFailure())
    {
      return error_code;
    }

    success = error_code.succeeded();
    if (!success)
    {
      ROS_INFO("planning failed for last random pose, retrying...");
    }
  }

  if (success)
  {
    ROS_INFO("moved to random position");
    resp.pose = random_pose;
  }
  else
  {
    ROS_WARN("failed to move to random position");
    return RLLErrorCode::NO_RANDOM_POSITION_FOUND;
  }

  return RLLErrorCode::SUCCESS;
}

bool RLLMoveIface::pickPlaceSrv(rll_msgs::PickPlace::Request& req, rll_msgs::PickPlace::Response& resp)
{
  return controlledMovementExecution(req, resp, PICK_PLACE_SRV_NAME, &RLLMoveIface::pickPlace);
}

RLLErrorCode RLLMoveIface::pickPlace(rll_msgs::PickPlace::Request& req, rll_msgs::PickPlace::Response& /*resp*/)
{
  RLLErrorCode error_code;
  geometry_msgs::Pose start = manip_move_group_.getCurrentPose().pose;

  if (!poseGoalTooClose(start, req.pose_above))
  {
    ROS_INFO("Moving above target");
    error_code = moveToGoalLinear(req.pose_above);
    if (error_code.failed())
    {
      ROS_WARN("Moving above target failed");
      return error_code;
    }
  }

  ROS_INFO("Moving to grip position");
  error_code = moveToGoalLinear(req.pose_grip);
  if (error_code.failed())
  {
    ROS_WARN("Moving to grip position failed");
    return error_code;
  }

  if (req.gripper_close != 0u)
  {
    attachGraspObject(req.grasp_object);
    error_code = closeGripper();
  }
  else
  {
    error_code = openGripper();
    // TODO(uieai): still detach if opening the gripper fails?
    detachGraspObject(req.grasp_object);
  }

  if (error_code.failed())
  {
    ROS_WARN("Opening or closing the gripper failed");
    return error_code;
  }

  ROS_INFO("Moving back above grip position");
  error_code = moveToGoalLinear(req.pose_above);
  if (error_code.failed())
  {
    ROS_WARN("Moving back above target failed");
  }

  return error_code;
}

bool RLLMoveIface::moveLinSrv(rll_msgs::MoveLin::Request& req, rll_msgs::MoveLin::Response& resp)
{
  return controlledMovementExecution(req, resp, MOVE_LIN_SRV_NAME, &RLLMoveIface::moveLin);
}

RLLErrorCode RLLMoveIface::moveLin(rll_msgs::MoveLin::Request& req, rll_msgs::MoveLin::Response& /*resp*/)
{
  RLLErrorCode error_code = poseGoalInCollision(req.pose);
  if (error_code.failed())
  {
    return error_code;
  }

  // moveLin service calls are disallowed to use cartesian_time_parametrization
  return moveToGoalLinear(req.pose, false);
}

bool RLLMoveIface::moveLinElbSrv(rll_msgs::MoveLinElb::Request& req, rll_msgs::MoveLinElb::Response& resp)
{
  return controlledMovementExecution(req, resp, MOVE_LIN_ELB_SRV_NAME, &RLLMoveIface::moveLinElb);
}

RLLErrorCode RLLMoveIface::moveLinElb(rll_msgs::MoveLinElb::Request& req, rll_msgs::MoveLinElb::Response& /*resp*/)
{
  bool success;
  std::vector<double> seed;
  double elb = req.elbow_angle;
  int dir = static_cast<int>(req.direction);
  const double JUMP_THRESHOLD = 4.5;

  if (elb < -M_PI || elb > M_PI)
  {
    ROS_WARN("requested elbow angle is out of range [-Pi,Pi]");
    return RLLErrorCode::INVALID_INPUT;
  }
  ROS_INFO_STREAM("Linear motion requested with elbow angle: " << elb);

  manip_move_group_.getCurrentState()->copyJointGroupPositions(manip_joint_model_group_, seed);

  std::shared_ptr<const rll_moveit_analytical_kinematics::RLLMoveItAnalyticalKinematicsPlugin> kinematics_plugin;
  if (!getKinematicsSolver(kinematics_plugin))
  {
    return RLLErrorCode::MOVEIT_PLANNING_FAILED;
  }

  // get elbow angle in start pose
  std::vector<std::string> link_names;
  std::vector<geometry_msgs::Pose> poses;
  double elb_start;
  kinematics_plugin->getPositionFKelb(link_names, seed, poses, elb_start);

  // calculate waypoints
  std::vector<geometry_msgs::Pose> waypoints;
  std::vector<double> waypoints_elb;

  interpolatePosesLinear(manip_move_group_.getCurrentPose().pose, req.pose, waypoints);
  transformPosesForIK(waypoints);
  interpolateElbLinear(elb_start, elb, dir, waypoints.size(), waypoints_elb);

  // calculate trajectory
  std::vector<robot_state::RobotStatePtr> traj;
  moveit_msgs::MoveItErrorCodes error_code;
  double last_valid_percentage = 0.0;

  kinematics_plugin->getPathIKelb(waypoints, waypoints_elb, seed, traj, manip_joint_model_group_, error_code,
                                  last_valid_percentage);

  // test for jump_threshold
  moveit::core::JumpThreshold thresh(JUMP_THRESHOLD);
  last_valid_percentage *= getCurrentRobotState().testJointSpaceJump(manip_joint_model_group_, traj, thresh);

  if (last_valid_percentage < 1 && last_valid_percentage > 0)
  {  // TODO(updim): visualize path until collision
    ROS_ERROR("only achieved to compute %f %% of the requested path", last_valid_percentage * 100.0);
    return RLLErrorCode::ONLY_PARTIAL_PATH_PLANNED;
  }
  if (last_valid_percentage <= 0)
  {
    ROS_ERROR("path planning completely failed");
    return RLLErrorCode::MOVEIT_PLANNING_FAILED;
  }

  // time trajectory
  robot_trajectory::RobotTrajectory rt(manip_model_, manip_move_group_.getName());

  for (int i = 0; i < waypoints.size(); i++)
  {
    rt.addSuffixWayPoint(traj[i], 0.0);
  }

  trajectory_processing::IterativeParabolicTimeParameterization time_param;
  time_param.computeTimeStamps(rt, 1.0);

  moveit_msgs::RobotTrajectory trajectory;
  rt.getRobotTrajectoryMsg(trajectory);

  // check for collisions
  if (!planning_scene_->isPathValid(rt))
  {  // TODO(updim): maybe output collision state
    ROS_ERROR("path colliding");
    return RLLErrorCode::ONLY_PARTIAL_PATH_PLANNED;
  }

  // moveLinElb service calls are disallowed to use cartesian_time_parametrization
  return runLinearTrajectory(trajectory, false);
}

bool RLLMoveIface::movePTPSrv(rll_msgs::MovePTP::Request& req, rll_msgs::MovePTP::Response& resp)
{
  return controlledMovementExecution(req, resp, MOVE_PTP_SRV_NAME, &RLLMoveIface::movePTP);
}

RLLErrorCode RLLMoveIface::movePTP(rll_msgs::MovePTP::Request& req, rll_msgs::MovePTP::Response& /*resp*/)
{
  manip_move_group_.setStartStateToCurrentState();
  bool success = manip_move_group_.setPoseTarget(req.pose);
  if (!success)
  {
    return RLLErrorCode::INVALID_TARGET_POSE;
  }

  RLLErrorCode error_code = poseGoalInCollision(req.pose);
  if (error_code.failed())
  {
    return error_code;
  }

  return runPTPTrajectory(manip_move_group_);
}

bool RLLMoveIface::movePTPelbSrv(rll_msgs::MovePTPelb::Request& req, rll_msgs::MovePTPelb::Response& resp)
{
  return controlledMovementExecution(req, resp, MOVE_PTP_ELB_SRV_NAME, &RLLMoveIface::movePTPelb);
}

RLLErrorCode RLLMoveIface::movePTPelb(rll_msgs::MovePTPelb::Request& req, rll_msgs::MovePTPelb::Response& /*resp*/)
{
  bool success;
  std::vector<double> seed;
  std::vector<double> sol;
  double elb = req.elbow_angle;

  if (elb < -M_PI || elb > M_PI)
  {
    ROS_WARN("requested elbow angle is out of range [-Pi,Pi]");
    return RLLErrorCode::INVALID_INPUT;
  }
  ROS_INFO_STREAM("PTP motion requested with elbow angle: " << elb);

  manip_move_group_.getCurrentState()->copyJointGroupPositions(manip_joint_model_group_, seed);

  std::shared_ptr<const rll_moveit_analytical_kinematics::RLLMoveItAnalyticalKinematicsPlugin> kinematics_plugin;
  if (!getKinematicsSolver(kinematics_plugin))
  {
    return RLLErrorCode::MOVEIT_PLANNING_FAILED;
  }

  // Transformations between frames
  tf::Transform world_to_ee, base_to_tip;
  geometry_msgs::Pose pose_tip;

  tf::poseMsgToTF(req.pose, world_to_ee);
  base_to_tip = base_to_world_ * world_to_ee * ee_to_tip_;
  tf::poseTFToMsg(base_to_tip, pose_tip);

  // get solution
  moveit_msgs::MoveItErrorCodes error_code;
  if (!kinematics_plugin->getPositionIKelb(pose_tip, seed, sol, error_code, elb))
  {
    ROS_ERROR("Inverse kinematics calculation failed");
    return RLLErrorCode::INVALID_TARGET_POSE;
  }

  manip_move_group_.setStartStateToCurrentState();
  success = manip_move_group_.setJointValueTarget(sol);

  if (!success)
  {
    ROS_ERROR("requested joint values are out of range");
    return RLLErrorCode::JOINT_VALUES_OUT_OF_RANGE;
  }

  return runPTPTrajectory(manip_move_group_);
}

bool RLLMoveIface::moveJointsSrv(rll_msgs::MoveJoints::Request& req, rll_msgs::MoveJoints::Response& resp)
{
  return controlledMovementExecution(req, resp, MOVE_JOINTS_SRV_NAME, &RLLMoveIface::moveJoints);
}

RLLErrorCode RLLMoveIface::moveJoints(rll_msgs::MoveJoints::Request& req, rll_msgs::MoveJoints::Response& /*resp*/)
{
  bool success;
  std::vector<double> joints(7);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  joints[0] = req.joint_1;
  joints[1] = req.joint_2;
  joints[2] = req.joint_3;
  joints[3] = req.joint_4;
  joints[4] = req.joint_5;
  joints[5] = req.joint_6;
  joints[6] = req.joint_7;

  manip_move_group_.setStartStateToCurrentState();

  success = manip_move_group_.setJointValueTarget(joints);
  if (!success)
  {
    ROS_ERROR("requested joint values are out of range");
    return RLLErrorCode::JOINT_VALUES_OUT_OF_RANGE;
  }

  return runPTPTrajectory(manip_move_group_);
}

bool RLLMoveIface::getCurrentJointValuesSrv(rll_msgs::GetJointValues::Request& /*req*/,
                                            rll_msgs::GetJointValues::Response& resp)
{
  RLLErrorCode error_code = beforeNonMovementServiceCall(RLLMoveIface::GET_JOINT_VALUES_SRV_NAME);

  if (error_code.succeeded())
  {
    std::vector<double> joints = manip_move_group_.getCurrentJointValues();
    resp.joint_1 = joints[0];
    resp.joint_2 = joints[1];
    resp.joint_3 = joints[2];
    resp.joint_4 = joints[3];
    resp.joint_5 = joints[4];
    resp.joint_6 = joints[5];
    resp.joint_7 = joints[6];
  }

  error_code = afterNonMovementServiceCall(RLLMoveIface::GET_JOINT_VALUES_SRV_NAME, error_code).value();
  resp.error_code = error_code.value();
  resp.success = error_code.succeeded();
  return true;
}

bool RLLMoveIface::getCurrentPoseSrv(rll_msgs::GetPose::Request& /*req*/, rll_msgs::GetPose::Response& resp)
{
  RLLErrorCode error_code = beforeNonMovementServiceCall(RLLMoveIface::GET_POSE_SRV_NAME);

  if (error_code.succeeded())
  {
    resp.pose = manip_move_group_.getCurrentPose().pose;
  }

  error_code = afterNonMovementServiceCall(RLLMoveIface::GET_POSE_SRV_NAME, error_code);
  resp.error_code = error_code.value();
  resp.success = error_code.succeeded();
  return true;
}

RLLErrorCode RLLMoveIface::runPTPTrajectory(moveit::planning_interface::MoveGroupInterface& move_group,
                                            bool for_gripper)
{
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode moveit_error_code;
  bool success;

  moveit_error_code = move_group.plan(my_plan);
  RLLErrorCode error_code = convertMoveItErrorCode(moveit_error_code);
  if (error_code.failed())
  {
    ROS_WARN("MoveIt planning failed: error code %s", stringifyMoveItErrorCodes(moveit_error_code));
    return error_code;
  }

  if (!for_gripper)
  {
    error_code = checkTrajectory(my_plan.trajectory_);
    if (error_code.failed())
    {
      return error_code;
    }

    success = modifyPtpTrajectory(my_plan.trajectory_);
    if (!success)
    {
      return RLLErrorCode::TRAJECTORY_MODIFICATION_FAILED;
    }
  }

  moveit_error_code = move_group.execute(my_plan);
  error_code = convertMoveItErrorCode(moveit_error_code);
  if (error_code.failed())
  {
    ROS_WARN("MoveIt plan execution failed: error code %s", stringifyMoveItErrorCodes(moveit_error_code));
    return error_code;
  }

  return RLLErrorCode::SUCCESS;
}

RLLErrorCode RLLMoveIface::moveToGoalLinear(const geometry_msgs::Pose& goal, bool cartesian_time_parametrization)
{
  moveit_msgs::RobotTrajectory trajectory;
  std::vector<geometry_msgs::Pose> waypoints;
  const double EEF_STEP = 0.0005;
  const double JUMP_THRESHOLD = 4.5;
  const double MIN_LIN_MOVEMENT_DISTANCE = 0.005;
  bool success;

  float distance = distanceToCurrentPosition(goal);
  if (distance < MIN_LIN_MOVEMENT_DISTANCE)
  {
    ROS_WARN("Linear motions that cover a distance of less than 5 mm are currently not supported."
             " Please use the 'move_ptp' service instead.");

    return RLLErrorCode::TOO_FEW_WAYPOINTS;
  }

  manip_move_group_.setStartStateToCurrentState();
  waypoints.push_back(goal);

  moveit::planning_interface::MoveItErrorCode moveit_error_code;
  double achieved =
      manip_move_group_.computeCartesianPath(waypoints, EEF_STEP, JUMP_THRESHOLD, trajectory, true, &moveit_error_code);

  if (achieved > 0 && achieved < 1)
  {
    ROS_ERROR("only achieved to compute %f of the requested path", achieved);
    return RLLErrorCode::ONLY_PARTIAL_PATH_PLANNED;
  }
  if (achieved <= 0)
  {
    ROS_ERROR("path planning completely failed, error code %s", stringifyMoveItErrorCodes(moveit_error_code));
    return RLLErrorCode::MOVEIT_PLANNING_FAILED;
  }

  return runLinearTrajectory(trajectory, cartesian_time_parametrization);
}

RLLErrorCode RLLMoveIface::runLinearTrajectory(const moveit_msgs::RobotTrajectory& trajectory,
                                               bool cartesian_time_parametrization)
{
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success;
  moveit::planning_interface::MoveItErrorCode moveit_error_code;

  my_plan.trajectory_ = trajectory;

  RLLErrorCode error_code = checkTrajectory(my_plan.trajectory_);
  if (error_code.failed())
  {
    return error_code;
  }

  // time parametrization happens in joint space by default
  if (cartesian_time_parametrization)
  {
    success = modifyLinTrajectory(my_plan.trajectory_);
    if (!success)
    {
      return RLLErrorCode::TRAJECTORY_MODIFICATION_FAILED;
    }
  }

  moveit_error_code = manip_move_group_.execute(my_plan);
  error_code = convertMoveItErrorCode(moveit_error_code);
  if (error_code.failed())
  {
    ROS_WARN("MoveIt plan execution failed: error code %s", stringifyMoveItErrorCodes(moveit_error_code));
    return error_code;
  }

  return RLLErrorCode::SUCCESS;
}

RLLErrorCode RLLMoveIface::checkTrajectory(moveit_msgs::RobotTrajectory& trajectory)
{
  if (trajectory.joint_trajectory.points.size() < 3)
  {
    ROS_WARN("trajectory has less than 3 points");
    return RLLErrorCode::TOO_FEW_WAYPOINTS;
  }

  std::vector<double> start = trajectory.joint_trajectory.points[0].positions;
  std::vector<double> goal = trajectory.joint_trajectory.points.back().positions;

  if (jointsGoalTooClose(start, goal))
  {
    ROS_WARN("trajectory: start state too close to goal state");
    return RLLErrorCode::GOAL_TOO_CLOSE_TO_START;
  }
  if (jointsGoalInCollision(goal))
  {
    return RLLErrorCode::GOAL_IN_COLLISION;
  }

  return RLLErrorCode::SUCCESS;
}

std::vector<double> RLLMoveIface::getJointValuesFromNamedTarget(const std::string& name)
{
  std::vector<double> goal;
  std::map<std::string, double> home_pose = manip_move_group_.getNamedTargetValues(name);

  for (const auto& it : home_pose)
  {
    goal.push_back(it.second);
  }
  return goal;
}

bool RLLMoveIface::jointsGoalTooClose(const std::vector<double>& start, const std::vector<double>& goal)
{
  float distance = 0.0;
  for (int i = 0; i < start.size(); ++i)
  {
    distance += fabs(start[i] - goal[i]);
  }

  return (distance < 0.01);
}

bool RLLMoveIface::poseGoalTooClose(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal)
{
  std::vector<double> start_joints, goal_joints;
  moveit_msgs::MoveItErrorCodes error_code;
  kinematics::KinematicsBaseConstPtr solver =
      manip_move_group_.getRobotModel()->getJointModelGroup(manip_move_group_.getName())->getSolverInstance();
  std::vector<double> seed = manip_move_group_.getCurrentJointValues();

  tf::Transform world_to_ee, base_to_tip;
  geometry_msgs::Pose pose_tip;

  tf::poseMsgToTF(start, world_to_ee);
  base_to_tip = base_to_world_ * world_to_ee * ee_to_tip_;
  tf::poseTFToMsg(base_to_tip, pose_tip);
  if (!solver->searchPositionIK(pose_tip, seed, 0.1, start_joints, error_code))
  {
    ROS_WARN("start pose for goal distance check invalid: error code %s", stringifyMoveItErrorCodes(error_code));
    return true;
  }

  tf::poseMsgToTF(goal, world_to_ee);
  base_to_tip = base_to_world_ * world_to_ee * ee_to_tip_;
  tf::poseTFToMsg(base_to_tip, pose_tip);
  if (!solver->searchPositionIK(pose_tip, seed, 0.1, goal_joints, error_code))
  {
    ROS_WARN("goal pose for goal distance check invalid: error code %s", stringifyMoveItErrorCodes(error_code));
    return true;
  }

  if (jointsGoalTooClose(start_joints, goal_joints))
  {
    ROS_WARN("goal joint values too close to start joint values");
    return true;
  }

  return false;
}

bool RLLMoveIface::jointsGoalInCollision(const std::vector<double>& goal)
{
  robot_state::RobotState goal_state = getCurrentRobotState();
  goal_state.setJointGroupPositions(manip_joint_model_group_, goal);
  if (stateInCollision(goal_state))
  {
    ROS_WARN("robot would be in collision for goal pose");
    return true;
  }

  return false;
}

RLLErrorCode RLLMoveIface::poseGoalInCollision(const geometry_msgs::Pose& goal)
{
  robot_state::RobotState goal_state = getCurrentRobotState();
  if (!goal_state.setFromIK(manip_joint_model_group_, goal, manip_move_group_.getEndEffectorLink()))
  {
    ROS_WARN("no IK solution found for given goal pose");
    return RLLErrorCode::NO_IK_SOLUTION_FOUND;
  }

  if (stateInCollision(goal_state))
  {
    ROS_WARN("robot would be in collision for given goal pose");
    return RLLErrorCode::GOAL_IN_COLLISION;
  }

  return RLLErrorCode::SUCCESS;
}

robot_state::RobotState RLLMoveIface::getCurrentRobotState(bool wait_for_state)
{
  if (wait_for_state)
  {
    planning_scene_monitor_->waitForCurrentRobotState(ros::Time::now());
  }

  planning_scene_monitor_->requestPlanningSceneState("get_planning_scene");
  planning_scene_monitor::LockedPlanningSceneRW planning_scene_rw(planning_scene_monitor_);
  planning_scene_rw->getCurrentStateNonConst().update();
  return planning_scene_rw->getCurrentState();
}

bool RLLMoveIface::stateInCollision(robot_state::RobotState& state)
{
  state.update(true);
  collision_detection::CollisionRequest request;
  request.distance = true;

  collision_detection::CollisionResult result;
  result.clear();
  planning_scene_->checkCollision(request, result, state, acm_);

  // There is either a collision or the distance between the robot
  // and the nearest collision object is less than 1mm.
  // Positions that are that close to a collision are disallowed
  // because the robot may end up being in collision when it
  // moves into the goal pose and ends up in a slightly different
  // position.
  return (result.collision || (result.distance >= 0.0 && result.distance < 0.001));
}

void RLLMoveIface::disableCollision(const std::string& link_1, const std::string& link_2)
{
  planning_scene_monitor::LockedPlanningSceneRW planning_scene_rw(planning_scene_monitor_);
  planning_scene_rw->getAllowedCollisionMatrixNonConst().setEntry(link_1, link_2, true);
  // we need a local copy because checkCollision doesn't automatically use the
  // updated collision matrix from the planning scene
  acm_ = planning_scene_rw->getAllowedCollisionMatrixNonConst();
}

RLLErrorCode RLLMoveIface::resetToHome()
{
  if (!manipCurrentStateAvailable())
  {
    return RLLErrorCode::MANIPULATOR_NOT_AVAILABLE;
  }

  std::vector<double> start = manip_move_group_.getCurrentJointValues();
  std::vector<double> goal = getJointValuesFromNamedTarget(HOME_TARGET_NAME);

  if (jointsGoalTooClose(start, goal))
  {
    // this is acceptable since we want to move here anyways -> skip movement
    ROS_INFO("resetToHome: no movement required, already at target position");
  }
  else
  {
    manip_move_group_.setStartStateToCurrentState();
    manip_move_group_.setNamedTarget(HOME_TARGET_NAME);

    RLLErrorCode error_code = runPTPTrajectory(manip_move_group_);
    if (error_code.failed())
    {
      return error_code;
    }
  }

  if (!no_gripper_attached_)
  {
    RLLErrorCode error_code = openGripper();
    if (error_code.failed())
    {
      return error_code;
    }
  }

  return RLLErrorCode::SUCCESS;
}

bool RLLMoveIface::attachGraspObject(const std::string& object_id)
{
  moveit_msgs::CollisionObject remove_object;

  if (object_id.empty())
  {
    return true;
  }

  ROS_INFO("attaching grasp object '%s'", object_id.c_str());

  std::map<std::string, moveit_msgs::CollisionObject> objects =
      planning_scene_interface_.getObjects(std::vector<std::string>{ object_id });

  if (!objects.empty())
  {
    remove_object = objects[object_id];
  }
  else
  {
    ROS_ERROR("object not found");
    return false;
  }

  if (remove_object.id != object_id)
  {
    ROS_ERROR("The found grasp object is not the right one");
    return false;
  }

  remove_object.operation = remove_object.REMOVE;

  planning_scene_interface_.applyCollisionObject(remove_object);

  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = manip_move_group_.getEndEffectorLink();
  attached_object.object = remove_object;
  attached_object.object.operation = attached_object.object.ADD;
  attached_object.touch_links =
      std::vector<std::string>{ ns_ + "_gripper_finger_left", ns_ + "_gripper_finger_right", "table" };
  planning_scene_interface_.applyAttachedCollisionObject(attached_object);

  return true;
}

bool RLLMoveIface::detachGraspObject(const std::string& object_id)
{
  moveit_msgs::AttachedCollisionObject remove_object;

  if (object_id.empty())
  {
    return true;
  }

  ROS_INFO("detaching grasp object '%s'", object_id.c_str());

  std::map<std::string, moveit_msgs::AttachedCollisionObject> objects =
      planning_scene_interface_.getAttachedObjects(std::vector<std::string>{ object_id });

  if (!objects.empty())
  {
    remove_object = objects[object_id];
  }
  else
  {
    ROS_ERROR("object not found");
    return false;
  }

  if (remove_object.object.id != object_id)
  {
    ROS_ERROR("The found grasp object is not the right one");
    return false;
  }

  remove_object.object.operation = remove_object.object.REMOVE;

  planning_scene_interface_.applyAttachedCollisionObject(remove_object);

  moveit_msgs::CollisionObject detached_object;
  detached_object = remove_object.object;
  detached_object.operation = detached_object.ADD;
  planning_scene_interface_.applyCollisionObject(detached_object);
  // TODO(uieai): figure out if this is really needed
  // occasionally, there seems to be a race condition with subsequent planning requests
  ros::Duration(0.1).sleep();

  return true;
}

void RLLMoveIface::interpolatePosesLinear(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end,
                                          std::vector<geometry_msgs::Pose>& waypoints)
{
  // most parts of code for cartesian interpolation from moveit's computeCartesianPath()
  const double EEF_STEP = 0.0005;

  Eigen::Isometry3d start_pose;
  tf::poseMsgToEigen(start, start_pose);

  Eigen::Isometry3d target_pose;
  tf::poseMsgToEigen(end, target_pose);

  Eigen::Quaterniond start_quaternion(start_pose.rotation());
  Eigen::Quaterniond target_quaternion(target_pose.rotation());

  // double rotation_distance = start_quaternion.angularDistance(target_quaternion);
  double translation_distance = (target_pose.translation() - start_pose.translation()).norm();

  // decide how many steps we will need for this trajectory
  std::size_t translation_steps = 0;
  if (EEF_STEP > 0.0)
  {
    translation_steps = floor(translation_distance / EEF_STEP);
  }

  // At least 30 steps even at constant pose for nullspace interpolation
  std::size_t steps = translation_steps + 1;
  if (steps < 30)  // in moveit 10 = MIN_STEPS_FOR_JUMP_THRESH
  {
    steps = 30;
  }

  waypoints.clear();
  waypoints.push_back(start);

  geometry_msgs::Pose tmp;

  for (std::size_t i = 1; i <= steps; ++i)  // slerp-interpolation
  {
    double percentage = static_cast<double>(i) / static_cast<double>(steps);

    Eigen::Isometry3d pose(start_quaternion.slerp(percentage, target_quaternion));
    pose.translation() = percentage * target_pose.translation() + (1 - percentage) * start_pose.translation();

    tf::poseEigenToMsg(pose, tmp);
    waypoints.push_back(tmp);
  }
}

void RLLMoveIface::interpolateElbLinear(const double start, const double end, const int dir, const int n,
                                        std::vector<double>& elb)
{
  double step_size_elb = (end - start) / (n - 1);

  // calculate step_size depending on direction and number of points
  if (dir == 1 && end < start)
  {
    step_size_elb = (2 * M_PI + end - start) / (n - 1);
  }
  else if (dir == -1 && end > start)
  {
    step_size_elb = (-2 * M_PI + end - start) / (n - 1);
  }

  // fill vector elb
  elb.clear();
  for (int i = 0; i < n; i++)
  {
    elb.push_back(start + i * step_size_elb);
  }
}

void RLLMoveIface::transformPosesForIK(std::vector<geometry_msgs::Pose>& waypoints)
{
  tf::Transform world_to_ee;
  tf::Transform base_to_tip;
  for (auto& waypoint : waypoints)
  {
    tf::poseMsgToTF(waypoint, world_to_ee);
    base_to_tip = base_to_world_ * world_to_ee * ee_to_tip_;
    tf::poseTFToMsg(base_to_tip, waypoint);
  }
}

bool RLLMoveIface::getKinematicsSolver(
    std::shared_ptr<const rll_moveit_analytical_kinematics::RLLMoveItAnalyticalKinematicsPlugin>& kinematics_plugin)
{
  // Load instance of solver and kinematics plugin
  const kinematics::KinematicsBaseConstPtr& solver = manip_joint_model_group_->getSolverInstance();
  kinematics_plugin =
      dynamic_pointer_cast<const rll_moveit_analytical_kinematics::RLLMoveItAnalyticalKinematicsPlugin>(solver);
  if (!kinematics_plugin)
  {
    ROS_ERROR("service only available using RLLMoveItAnalyticalKinematicsPlugin");
    return false;
  }
  return true;
}

bool RLLMoveIface::initConstTransforms()
{
  // Static Transformations between frames
  const kinematics::KinematicsBaseConstPtr& solver = manip_joint_model_group_->getSolverInstance();
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  geometry_msgs::TransformStamped ee_to_tip_stamped, base_to_world_stamped;
  std::string world_frame = manip_move_group_.getPlanningFrame();
#if ROS_VERSION_MINIMUM(1, 14, 3)  // Melodic
                                   // leave world_frame as is
#else                              // Kinetic and older
  world_frame.erase(0, 1);  // remove slash
#endif
  try
  {
    ee_to_tip_stamped = tf_buffer.lookupTransform(manip_move_group_.getEndEffectorLink(), solver->getTipFrame(),
                                                  ros::Time(0), ros::Duration(1.0));
    base_to_world_stamped =
        tf_buffer.lookupTransform(solver->getBaseFrame(), world_frame, ros::Time(0), ros::Duration(1.0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_FATAL("%s", ex.what());
    abortDueToCriticalFailure();
    return false;
  }

  tf::transformMsgToTF(ee_to_tip_stamped.transform, ee_to_tip_);
  tf::transformMsgToTF(base_to_world_stamped.transform, base_to_world_);
  return true;
}

float RLLMoveIface::distanceToCurrentPosition(const geometry_msgs::Pose& pose)
{
  geometry_msgs::Pose current_pose = manip_move_group_.getCurrentPose().pose;

  return sqrt(pow(current_pose.position.x - pose.position.x, 2) + pow(current_pose.position.y - pose.position.y, 2) +
              pow(current_pose.position.z - pose.position.z, 2));
}

bool waitForMoveGroupAction()
{
  // to ensure that the moveit setup is completely loaded wait for move_group action to become available
  ROS_INFO("Waiting for the 'move_group' action to become available");

  actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> ac("move_group", true);
  if (!ac.waitForServer(ros::Duration(10)))
  {
    ROS_FATAL("'move_group' action is not available! Did you launch all required nodes?");
    return false;
  }
  ROS_INFO("done waiting");

  ros::Duration(1).sleep();
  return true;
}

/*
 * Local Variables:
 * c-file-style: "google"
 * End:
 */
