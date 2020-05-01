/*
 * This file is part of the Robot Learning Lab SDK
 *
 * Copyright (C) 2018-2020 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
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

#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_listener.h>

#include <rll_move/move_iface_planning.h>

const double RLLMoveIfacePlanning::DEFAULT_VELOCITY_SCALING_FACTOR = 0.4;
const double RLLMoveIfacePlanning::DEFAULT_ACCELERATION_SCALING_FACTOR = 0.4;

const double RLLMoveIfacePlanning::DEFAULT_LINEAR_EEF_STEP = 0.0005;
const double RLLMoveIfacePlanning::DEFAULT_LINEAR_JUMP_THRESHOLD = 4.5;
const double RLLMoveIfacePlanning::LINEAR_MIN_STEPS_FOR_JUMP_THRESH = 10;

const std::string RLLMoveIfacePlanning::MANIP_PLANNING_GROUP = "manipulator";
const std::string RLLMoveIfacePlanning::GRIPPER_PLANNING_GROUP = "gripper";

const std::string RLLMoveIfacePlanning::HOME_TARGET_NAME = "home_bow";
const std::string RLLMoveIfacePlanning::GRIPPER_OPEN_TARGET_NAME = "gripper_open";
const std::string RLLMoveIfacePlanning::GRIPPER_CLOSE_TARGET_NAME = "gripper_close";

RLLMoveIfacePlanning::RLLMoveIfacePlanning()
  : manip_move_group_(MANIP_PLANNING_GROUP), gripper_move_group_(GRIPPER_PLANNING_GROUP)
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

  ros::param::get("move_group/trajectory_execution/allowed_start_tolerance", allowed_start_tolerance_);

  manip_move_group_.setPlannerId("RRTConnectkConfigDefault");
  manip_move_group_.setPlanningTime(2.0);
  manip_move_group_.setPoseReferenceFrame("world");
  gripper_move_group_.setPlannerId("RRTConnectkConfigDefault");
  gripper_move_group_.setPlanningTime(2.0);

  manip_move_group_.setMaxVelocityScalingFactor(DEFAULT_VELOCITY_SCALING_FACTOR);
  manip_move_group_.setMaxAccelerationScalingFactor(DEFAULT_ACCELERATION_SCALING_FACTOR);

  manip_model_ = manip_move_group_.getRobotModel();
  manip_joint_model_group_ = manip_model_->getJointModelGroup(manip_move_group_.getName());

  std::string ee_link = ns_ + "_gripper_link_ee";
  manip_move_group_.setEndEffectorLink(ee_link);

  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

  planning_scene_monitor_->requestPlanningSceneState("get_planning_scene");
  planning_scene_ = planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_);
  acm_ = planning_scene_->getAllowedCollisionMatrix();

  // startup checks, shutdown the node if something is wrong
  if (!isCollisionLinkAvailable() || !getKinematicsSolver() || !initConstTransforms())
  {
    ROS_FATAL("Startup checks failed, shutting the node down!");
    ros::shutdown();
  }
}

bool RLLMoveIfacePlanning::isCollisionLinkAvailable()
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

bool RLLMoveIfacePlanning::manipCurrentStateAvailable()
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

RLLErrorCode RLLMoveIfacePlanning::runPTPTrajectory(moveit::planning_interface::MoveGroupInterface* move_group,
                                                    bool for_gripper)
{
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode moveit_error_code;
  bool success;

  moveit_error_code = move_group->plan(my_plan);
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

    success = modifyPtpTrajectory(&my_plan.trajectory_);
    if (!success)
    {
      return RLLErrorCode::TRAJECTORY_MODIFICATION_FAILED;
    }
  }

  return execute(move_group, my_plan);
}

RLLErrorCode RLLMoveIfacePlanning::execute(moveit::planning_interface::MoveGroupInterface* move_group,
                                           const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
  moveit::planning_interface::MoveItErrorCode moveit_error_code;

  moveit_error_code = move_group->execute(plan);
  RLLErrorCode error_code = convertMoveItErrorCode(moveit_error_code);
  if (error_code.failed())
  {
    ROS_WARN("MoveIt plan execution failed: error code %s", stringifyMoveItErrorCodes(moveit_error_code));
    return error_code;
  }

  std::vector<double> last_point = plan.trajectory_.joint_trajectory.points.back().positions;
  bool identical = false;
  std::vector<double> current_point;
  ros::Rate r(200);
  ros::Duration timeout(2);
  ros::Time begin = ros::Time::now();
  while (!identical && ros::Time::now() - begin < timeout)
  {
    // current state in move_group is not uptodate with last state from planning scene, so fetch directly from planning
    // scene
    getCurrentRobotState().copyJointGroupPositions(manip_joint_model_group_, current_point);
    for (size_t i = 0; i < last_point.size(); ++i)
    {
      if (fabs(current_point[i] - last_point[i]) >= allowed_start_tolerance_)
      {
        r.sleep();
        break;
      }
      if (i == last_point.size() - 1)
      {
        identical = true;
      }
    }
  }

  if (!identical)
  {
    ROS_FATAL("desired goal state was not reached");
    return RLLErrorCode::EXECUTION_FAILED;
  }

  return RLLErrorCode::SUCCESS;
}

RLLErrorCode RLLMoveIfacePlanning::moveToGoalLinear(const geometry_msgs::Pose& goal,
                                                    bool cartesian_time_parametrization)
{
  moveit_msgs::RobotTrajectory trajectory;
  std::vector<geometry_msgs::Pose> waypoints;
  const double MIN_LIN_MOVEMENT_DISTANCE = 0.005;

  float distance = distanceToCurrentPosition(goal);
  if (distance < MIN_LIN_MOVEMENT_DISTANCE)
  {
    ROS_WARN("Linear motions that cover a distance of less than 5 mm are currently not supported."
             " Please use the 'move_ptp' service instead.");

    return RLLErrorCode::TOO_FEW_WAYPOINTS;
  }

  manip_move_group_.setStartStateToCurrentState();
  waypoints.push_back(goal);
  RLLErrorCode error_code = computeLinearPath(waypoints, &trajectory);
  if (error_code.failed())
  {
    return error_code;
  }

  return runLinearTrajectory(trajectory, cartesian_time_parametrization);
}

RLLErrorCode RLLMoveIfacePlanning::computeLinearPath(const std::vector<geometry_msgs::Pose>& waypoints,
                                                     moveit_msgs::RobotTrajectory* trajectory)
{
  moveit::planning_interface::MoveItErrorCode moveit_error_code;

  double achieved = manip_move_group_.computeCartesianPath(
      waypoints, DEFAULT_LINEAR_EEF_STEP, DEFAULT_LINEAR_JUMP_THRESHOLD, *trajectory, true, &moveit_error_code);

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

  if (trajectory->joint_trajectory.points.size() < LINEAR_MIN_STEPS_FOR_JUMP_THRESH)
  {
    ROS_ERROR("trajectory has not enough points to check for continuity, only got %lu",
              trajectory->joint_trajectory.points.size());
    return RLLErrorCode::TOO_FEW_WAYPOINTS;
  }

  return RLLErrorCode::SUCCESS;
}

RLLErrorCode RLLMoveIfacePlanning::runLinearTrajectory(const moveit_msgs::RobotTrajectory& trajectory,
                                                       bool cartesian_time_parametrization)
{
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success;

  my_plan.trajectory_ = trajectory;

  RLLErrorCode error_code = checkTrajectory(my_plan.trajectory_);
  if (error_code.failed())
  {
    return error_code;
  }

  // time parametrization happens in joint space by default
  if (cartesian_time_parametrization)
  {
    success = modifyLinTrajectory(&my_plan.trajectory_);
    if (!success)
    {
      return RLLErrorCode::TRAJECTORY_MODIFICATION_FAILED;
    }
  }
  else
  {
    success = modifyPtpTrajectory(&my_plan.trajectory_);
    if (!success)
    {
      return RLLErrorCode::TRAJECTORY_MODIFICATION_FAILED;
    }
  }

  return execute(&manip_move_group_, my_plan);
}

RLLErrorCode RLLMoveIfacePlanning::checkTrajectory(const moveit_msgs::RobotTrajectory& trajectory)
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

std::vector<double> RLLMoveIfacePlanning::getJointValuesFromNamedTarget(const std::string& name)
{
  std::vector<double> goal;
  std::map<std::string, double> home_pose = manip_move_group_.getNamedTargetValues(name);

  goal.reserve(RLL_NUM_JOINTS);
  for (const auto& it : home_pose)
  {
    goal.push_back(it.second);
  }
  return goal;
}

bool RLLMoveIfacePlanning::jointsGoalTooClose(const std::vector<double>& start, const std::vector<double>& goal)
{
  float distance = 0.0;
  for (unsigned int i = 0; i < start.size(); ++i)
  {
    distance += fabs(start[i] - goal[i]);
  }

  return (distance < 0.01);
}

bool RLLMoveIfacePlanning::poseGoalTooClose(const geometry_msgs::Pose& goal)
{
  std::vector<double> goal_joints;
  moveit_msgs::MoveItErrorCodes error_code;
  std::vector<double> seed = manip_move_group_.getCurrentJointValues();

  geometry_msgs::Pose pose_tip = goal;
  transformPoseForIK(&pose_tip);
  if (!kinematics_plugin_->searchPositionIK(pose_tip, seed, 0.1, goal_joints, error_code))
  {
    ROS_WARN("goal pose for goal distance check invalid: error code %s", stringifyMoveItErrorCodes(error_code));
    return true;
  }

  if (jointsGoalTooClose(seed, goal_joints))
  {
    ROS_WARN("goal joint values too close to start joint values");
    return true;
  }

  return false;
}

bool RLLMoveIfacePlanning::jointsGoalInCollision(const std::vector<double>& goal)
{
  robot_state::RobotState goal_state = getCurrentRobotState();
  goal_state.setJointGroupPositions(manip_joint_model_group_, goal);
  if (stateInCollision(&goal_state))
  {
    ROS_WARN("robot would be in collision for goal pose");
    return true;
  }

  return false;
}

RLLErrorCode RLLMoveIfacePlanning::poseGoalInCollision(const geometry_msgs::Pose& goal)
{
  robot_state::RobotState goal_state = getCurrentRobotState();
  // TODO(wolfgang): pass a GroupStateValidityCallbackFn function. Otherwise the inverse cannot verify
  //                 if pose is in collision
  //                 wrap stateInCollision() as GroupStateValidityCallbackFn
  if (!goal_state.setFromIK(manip_joint_model_group_, goal, manip_move_group_.getEndEffectorLink()))
  {
    ROS_WARN("no IK solution found for given goal pose");
    return RLLErrorCode::NO_IK_SOLUTION_FOUND;
  }

  if (stateInCollision(&goal_state))
  {
    ROS_WARN("robot would be in collision for given goal pose");
    return RLLErrorCode::GOAL_IN_COLLISION;
  }

  return RLLErrorCode::SUCCESS;
}

robot_state::RobotState RLLMoveIfacePlanning::getCurrentRobotState(bool wait_for_state)
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

bool RLLMoveIfacePlanning::stateInCollision(robot_state::RobotState* state)
{
  state->update(true);
  collision_detection::CollisionRequest request;
  request.distance = true;

  collision_detection::CollisionResult result;
  result.clear();
  planning_scene_->checkCollision(request, result, *state, acm_);

  // There is either a collision or the distance between the robot
  // and the nearest collision object is less than 1mm.
  // Positions that are that close to a collision are disallowed
  // because the robot may end up being in collision when it
  // moves into the goal pose and ends up in a slightly different
  // position.
  return (result.collision || (result.distance >= 0.0 && result.distance < 0.001));
}

void RLLMoveIfacePlanning::disableCollision(const std::string& link_1, const std::string& link_2)
{
  planning_scene_monitor::LockedPlanningSceneRW planning_scene_rw(planning_scene_monitor_);
  planning_scene_rw->getAllowedCollisionMatrixNonConst().setEntry(link_1, link_2, true);
  // we need a local copy because checkCollision doesn't automatically use the
  // updated collision matrix from the planning scene
  acm_ = planning_scene_rw->getAllowedCollisionMatrixNonConst();
}

bool RLLMoveIfacePlanning::attachGraspObject(const std::string& object_id)
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

bool RLLMoveIfacePlanning::detachGraspObject(const std::string& object_id)
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

RLLErrorCode RLLMoveIfacePlanning::computeLinearPathArmangle(const std::vector<geometry_msgs::Pose>& waypoints_pose,
                                                             const std::vector<double>& waypoints_arm_angles,
                                                             const std::vector<double>& ik_seed_state,
                                                             std::vector<robot_state::RobotStatePtr>* path)
{
  double last_valid_percentage = 0.0;
  getPathIK(waypoints_pose, waypoints_arm_angles, ik_seed_state, path, &last_valid_percentage);

  std::vector<robot_state::RobotStatePtr> traj;

  // test for jump_threshold
  moveit::core::JumpThreshold thresh(DEFAULT_LINEAR_JUMP_THRESHOLD);
  last_valid_percentage *= robot_state::RobotState::testJointSpaceJump(manip_joint_model_group_, *path, thresh);

  if (last_valid_percentage < 1.0 && last_valid_percentage > 0.0)
  {  // TODO(updim): visualize path until collision
    ROS_ERROR("only achieved to compute %f %% of the requested path", last_valid_percentage * 100.0);
    return RLLErrorCode::ONLY_PARTIAL_PATH_PLANNED;
  }
  if (last_valid_percentage <= 0.0)
  {
    ROS_ERROR("path planning completely failed");
    return RLLErrorCode::MOVEIT_PLANNING_FAILED;
  }

  return RLLErrorCode::SUCCESS;
}

void RLLMoveIfacePlanning::getPathIK(const std::vector<geometry_msgs::Pose>& waypoints_pose,
                                     const std::vector<double>& waypoints_arm_angles,
                                     const std::vector<double>& ik_seed_state,
                                     std::vector<robot_state::RobotStatePtr>* path, double* last_valid_percentage)
{
  robot_state::RobotState tmp_state(manip_model_);
  std::vector<double> sol(RLL_NUM_JOINTS);
  std::vector<double> seed_tmp = ik_seed_state;
  bool success;

  if (waypoints_pose.size() != waypoints_arm_angles.size())
  {
    ROS_ERROR("getPathIK: size of waypoints and arm angles vectors do not match");
    *last_valid_percentage = 0.0;
    return;
  }

  tmp_state.setToDefaultValues();

  for (size_t i = 0; i < waypoints_pose.size(); i++)
  {
    moveit_msgs::MoveItErrorCodes error_code;
    success = kinematics_plugin_->getPositionIKarmangle(waypoints_pose[i], seed_tmp, &sol, &error_code,
                                                        waypoints_arm_angles[i]);
    if (!success)
    {
      *last_valid_percentage = static_cast<double>(i) / static_cast<double>(waypoints_pose.size());
      return;
    }

    seed_tmp = sol;
    tmp_state.setJointGroupPositions(manip_joint_model_group_, sol);
    path->push_back(std::make_shared<moveit::core::RobotState>(tmp_state));
  }

  *last_valid_percentage = 1.0;
}

void RLLMoveIfacePlanning::interpolatePosesLinear(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end,
                                                  std::vector<geometry_msgs::Pose>* waypoints)
{
  // most parts of code for cartesian interpolation from moveit's computeCartesianPath()

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
  if (DEFAULT_LINEAR_EEF_STEP > 0.0)
  {
    translation_steps = floor(translation_distance / DEFAULT_LINEAR_EEF_STEP);
  }

  // At least 30 steps even at constant pose for nullspace interpolation
  std::size_t steps = translation_steps + 1;
  if (steps < 30)  // in moveit 10 = MIN_STEPS_FOR_JUMP_THRESH
  {
    steps = 30;
  }

  waypoints->clear();
  waypoints->push_back(start);

  geometry_msgs::Pose tmp;

  for (std::size_t i = 1; i <= steps; ++i)  // slerp-interpolation
  {
    double percentage = static_cast<double>(i) / static_cast<double>(steps);

    Eigen::Isometry3d pose(start_quaternion.slerp(percentage, target_quaternion));
    pose.translation() = percentage * target_pose.translation() + (1 - percentage) * start_pose.translation();

    tf::poseEigenToMsg(pose, tmp);
    waypoints->push_back(tmp);
  }
}

void RLLMoveIfacePlanning::interpolateArmangleLinear(const double start, const double end, const int dir, const int n,
                                                     std::vector<double>* arm_angles)
{
  double step_size_arm_angle;

  // calculate step_size depending on direction and number of points
  if (dir == 1 && end < start)
  {
    step_size_arm_angle = (2 * M_PI + end - start) / (n - 1);
  }
  else if (dir == -1 && end > start)
  {
    step_size_arm_angle = (-2 * M_PI + end - start) / (n - 1);
  }
  else
  {
    step_size_arm_angle = (end - start) / (n - 1);
  }

  // fill vector
  arm_angles->clear();
  for (int i = 0; i < n; i++)
  {
    arm_angles->push_back(start + i * step_size_arm_angle);
  }
}

void RLLMoveIfacePlanning::transformPoseForIK(geometry_msgs::Pose* pose)
{
  tf::Transform world_to_ee;
  tf::Transform base_to_tip;

  tf::poseMsgToTF(*pose, world_to_ee);
  base_to_tip = base_to_world_ * world_to_ee * ee_to_tip_;
  tf::poseTFToMsg(base_to_tip, *pose);
}

bool RLLMoveIfacePlanning::armangleInRange(double arm_angle)
{
  if (arm_angle < -M_PI || arm_angle > M_PI)
  {
    ROS_WARN("requested arm angle is out of range [-Pi,Pi]");
    return false;
  }

  return true;
}

bool RLLMoveIfacePlanning::getKinematicsSolver()
{
  // Load instance of solver and kinematics plugin
  const kinematics::KinematicsBaseConstPtr& solver = manip_joint_model_group_->getSolverInstance();
  kinematics_plugin_ = std::dynamic_pointer_cast<const rll_moveit_kinematics::RLLMoveItKinematicsPlugin>(solver);
  if (!kinematics_plugin_)
  {
    ROS_FATAL("RLLMoveItKinematicsPlugin could not be loaded");
    return false;
  }

  return true;
}

bool RLLMoveIfacePlanning::initConstTransforms()
{
  // Static Transformations between frames
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
    ee_to_tip_stamped = tf_buffer.lookupTransform(manip_move_group_.getEndEffectorLink(),
                                                  kinematics_plugin_->getTipFrame(), ros::Time(0), ros::Duration(1.0));
    base_to_world_stamped =
        tf_buffer.lookupTransform(kinematics_plugin_->getBaseFrame(), world_frame, ros::Time(0), ros::Duration(1.0));
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

float RLLMoveIfacePlanning::distanceToCurrentPosition(const geometry_msgs::Pose& pose)
{
  geometry_msgs::Pose current_pose = manip_move_group_.getCurrentPose().pose;

  return sqrt(pow(current_pose.position.x - pose.position.x, 2) + pow(current_pose.position.y - pose.position.y, 2) +
              pow(current_pose.position.z - pose.position.z, 2));
}

RLLErrorCode RLLMoveIfacePlanning::closeGripper()
{
  if (!manipCurrentStateAvailable())
  {
    return RLLErrorCode::MANIPULATOR_NOT_AVAILABLE;
  }
  ROS_INFO("Closing the gripper");

  gripper_move_group_.setStartStateToCurrentState();
  gripper_move_group_.setNamedTarget(GRIPPER_CLOSE_TARGET_NAME);

  RLLErrorCode error_code = runPTPTrajectory(&gripper_move_group_, true);
  if (error_code.failed())
  {
    ROS_INFO("Failed to close the gripper");
  }

  return error_code;
}

RLLErrorCode RLLMoveIfacePlanning::openGripper()
{
  if (!manipCurrentStateAvailable())
  {
    return RLLErrorCode::MANIPULATOR_NOT_AVAILABLE;
  }
  ROS_INFO("Opening the gripper");

  gripper_move_group_.setStartStateToCurrentState();
  gripper_move_group_.setNamedTarget(GRIPPER_OPEN_TARGET_NAME);

  RLLErrorCode error_code = runPTPTrajectory(&gripper_move_group_, true);
  if (error_code.failed())
  {
    ROS_INFO("Failed to open the gripper");
  }

  return error_code;
}

bool RLLMoveIfacePlanning::modifyLinTrajectory(moveit_msgs::RobotTrajectory* trajectory)
{
  // TODO(wolfgang): do ptp parametrization as long as we don't have a working cartesian parametrization
  ROS_WARN("cartesian time parametrization is not yet supported, applying PTP parametrization instead");
  return modifyPtpTrajectory(trajectory);
}
