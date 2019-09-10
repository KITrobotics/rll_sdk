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
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

const std::string RLLMoveIface::MANIP_PLANNING_GROUP = "manipulator";
const std::string RLLMoveIface::GRIPPER_PLANNING_GROUP = "gripper";

const std::string RLLMoveIface::IDLE_JOB_SRV_NAME = "job_idle";
const std::string RLLMoveIface::RUN_JOB_SRV_NAME = "job_env";

const std::string RLLMoveIface::ROBOT_READY_SRV_NAME = "robot_ready";
const std::string RLLMoveIface::MOVE_PTP_SRV_NAME = "move_ptp";
const std::string RLLMoveIface::MOVE_LIN_SRV_NAME = "move_lin";
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

  allowed_to_move_ = false;
}

void RLLMoveIface::runJobAction(const rll_msgs::JobEnvGoalConstPtr& goal, JobServer* as)
{
  rll_msgs::JobEnvResult result;
  // TODO(uieai): add some checks here. E.g:
  // - is a job currently running?
  // - validate if the caller is authenticated to run a job
  allowed_to_move_ = true;

  runJob(goal, result);

  // sanity check: if a job fails with an internal error than further operations should have been aborted
  if (result.job.status == rll_msgs::JobStatus::INTERNAL_ERROR && allowed_to_move_)
  {
    ROS_FATAL("Job resulted in an INTERNAL_ERROR, but movement is still allowed. This should not happen!");
    abortDueToCriticalFailure();
  }

  allowed_to_move_ = false;
  as->setSucceeded(result);
}

void RLLMoveIface::idleAction(const rll_msgs::JobEnvGoalConstPtr& /*goal*/, JobServer* as)
{
  rll_msgs::JobEnvResult result;

  RLLErrorCode error_code = idle();

  if (error_code.succeeded())
  {
    ROS_INFO("Idle succeeded!");
    result.job.status = rll_msgs::JobStatus::SUCCESS;
  }
  else
  {
    if (error_code.isRecoverableFailure())
    {
      ROS_FATAL("Idle resulted in a recoverable failure!");
    }
    else
    {
      ROS_FATAL("Idle resulted in a critical failure!");
    }

    result.job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
  }

  as->setSucceeded(result);
}

RLLErrorCode RLLMoveIface::idle()
{
  ROS_INFO("got idle request");
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
  RLLErrorCode error_code = resetToHome();
  resp.success = error_code.succeeded();  // NOLINT
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
  allowed_to_move_ = false;
}

void RLLMoveIface::handleFailureSeverity(const RLLErrorCode& error_code)
{
  // check if a error condition matches, if not assume critical failure

  // TODO(uieai): these checks require testing!!
  if (error_code.isInvalidInput())
  {
    ROS_WARN("A failure due to invalid user input occurred. error: %s", error_code.message());
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

RLLErrorCode RLLMoveIface::beforeMovementSrvChecks(const std::string& srv_name)
{
  ROS_INFO("service '%s' requested", srv_name.c_str());

  if (!allowed_to_move_)
  {
    ROS_WARN("Not allowed to send '%s' commands", srv_name.c_str());
    return RLLErrorCode::MOVEMENT_NOT_ALLOWED;
  }

  if (!manipCurrentStateAvailable())
  {
    return RLLErrorCode::MANIPULATOR_NOT_AVAILABLE;
  }

  return RLLErrorCode::SUCCESS;
}

template <class Request, class Response>
bool RLLMoveIface::controlledMovementExecution(Request& req, Response& resp, const std::string& srv_name,
                                               RLLErrorCode (RLLMoveIface::*move_func)(Request&, Response&))
{
  RLLErrorCode error_code = beforeMovementSrvChecks(srv_name);
  if (error_code.failed())
  {
    resp.success = false;
    resp.error_code = error_code.value();
    return true;
  }

  error_code = (this->*move_func)(req, resp);
  resp.error_code = error_code.value();
  resp.success = error_code.succeeded();

  if (error_code.failed())
  {
    ROS_WARN("'%s' service call failed!", srv_name.c_str());
    handleFailureSeverity(error_code);
  }

  return true;
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

    if (poseGoalInCollision(random_pose))
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

    RLLErrorCode error_code = runPTPTrajectory(manip_move_group_);
    // make sure nothing major went wrong. only repeat in case of noncritical errors
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
    error_code = runLinearTrajectory(req.pose_above);
    if (error_code.failed())
    {
      ROS_WARN("Moving above target failed");
      return error_code;
    }
  }

  ROS_INFO("Moving to grip position");
  error_code = runLinearTrajectory(req.pose_grip);
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
  error_code = runLinearTrajectory(req.pose_above);
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
  if (poseGoalInCollision(req.pose))
  {
    return RLLErrorCode::GOAL_IN_COLLISION;
  }

  // moveLin service calls are disallowed to use cartesian_time_parametrization
  return runLinearTrajectory(req.pose, false);
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
  std::vector<double> joints = manip_move_group_.getCurrentJointValues();
  resp.joint_1 = joints[0];
  resp.joint_2 = joints[1];
  resp.joint_3 = joints[2];
  resp.joint_4 = joints[3];
  resp.joint_5 = joints[4];
  resp.joint_6 = joints[5];
  resp.joint_7 = joints[6];
  return true;
}

bool RLLMoveIface::getCurrentPoseSrv(rll_msgs::GetPose::Request& /*req*/, rll_msgs::GetPose::Response& resp)
{
  resp.pose = manip_move_group_.getCurrentPose().pose;
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
  }

  success = modifyPtpTrajectory(my_plan.trajectory_);
  if (!success)
  {
    return RLLErrorCode::TRAJECTORY_MODIFICATION_FAILED;
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

RLLErrorCode RLLMoveIface::runLinearTrajectory(const geometry_msgs::Pose& goal, bool cartesian_time_parametrization)
{
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  std::vector<geometry_msgs::Pose> waypoints;
  const double EEF_STEP = 0.0005;
  const double JUMP_THRESHOLD = 4.5;
  bool success;

  manip_move_group_.setStartStateToCurrentState();
  waypoints.push_back(goal);

  moveit::planning_interface::MoveItErrorCode moveit_error_code;
  double achieved = manip_move_group_.computeCartesianPath(waypoints, EEF_STEP, JUMP_THRESHOLD, my_plan.trajectory_,
                                                           true, &moveit_error_code);

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

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  tf::Transform world_to_ee, base_to_tip, ee_to_tip, base_to_world;
  geometry_msgs::TransformStamped ee_to_tip_stamped, base_to_world_stamped;
  geometry_msgs::Pose pose_tip;
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
    ros::Duration(1.0).sleep();
    // TODO(uieai): is this mandatory? e.g. move_random uses this function as a test
    abortDueToCriticalFailure();
    return true;
  }

  tf::transformMsgToTF(ee_to_tip_stamped.transform, ee_to_tip);
  tf::transformMsgToTF(base_to_world_stamped.transform, base_to_world);

  tf::poseMsgToTF(start, world_to_ee);
  base_to_tip = base_to_world * world_to_ee * ee_to_tip;
  tf::poseTFToMsg(base_to_tip, pose_tip);
  if (!solver->searchPositionIK(pose_tip, seed, 0.1, start_joints, error_code))
  {
    ROS_WARN("start pose for goal distance check invalid: error code %s", stringifyMoveItErrorCodes(error_code));
    return true;
  }

  tf::poseMsgToTF(goal, world_to_ee);
  base_to_tip = base_to_world * world_to_ee * ee_to_tip;
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

bool RLLMoveIface::poseGoalInCollision(const geometry_msgs::Pose& goal)
{
  robot_state::RobotState goal_state = getCurrentRobotState();
  if (!goal_state.setFromIK(manip_joint_model_group_, goal, manip_move_group_.getEndEffectorLink()))
  {
    ROS_WARN("goal pose not valid");
    return true;
  }

  if (stateInCollision(goal_state))
  {
    ROS_WARN("robot would be in collision for goal pose");
    return true;
  }

  return false;
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
  // TODO(uieai): is it garanteeted, that distance == 0.0 is a collision?
  return (result.collision || (result.distance > 0.0 && result.distance < 0.001));
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

/*
 * Local Variables:
 * c-file-style: "google"
 * End:
 */
