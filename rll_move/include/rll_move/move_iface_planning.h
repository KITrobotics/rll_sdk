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

#ifndef RLL_MOVE_MOVE_IFACE_PLANNING_H
#define RLL_MOVE_MOVE_IFACE_PLANNING_H

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <rll_move/move_iface_error.h>
#include <rll_moveit_kinematics_plugin/moveit_kinematics_plugin.h>

class RLLMoveIfacePlanning
{
public:
  explicit RLLMoveIfacePlanning();
  virtual ~RLLMoveIfacePlanning() = default;

protected:
  static const std::string HOME_TARGET_NAME;

  static const double DEFAULT_VELOCITY_SCALING_FACTOR;
  static const double DEFAULT_ACCELERATION_SCALING_FACTOR;

  static const double DEFAULT_LINEAR_EEF_STEP;
  static const double DEFAULT_ROTATION_EEF_STEP;
  static const double DEFAULT_LINEAR_JUMP_THRESHOLD;
  static const size_t LINEAR_MIN_STEPS_FOR_JUMP_THRESH;

  // TODO(wolfgang): make these private
  std::string node_name_;
  moveit::planning_interface::MoveGroupInterface manip_move_group_;
  const robot_state::JointModelGroup* manip_joint_model_group_;
  std::shared_ptr<const rll_moveit_kinematics::RLLMoveItKinematicsPlugin> kinematics_plugin_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  moveit::core::RobotModelConstPtr manip_model_;
  bool no_gripper_attached_ = false;

  RLLErrorCode runPTPTrajectory(moveit::planning_interface::MoveGroupInterface* move_group, bool for_gripper = false);
  RLLErrorCode moveToGoalLinear(const geometry_msgs::Pose& goal, bool cartesian_time_parametrization = false);
  RLLErrorCode runLinearTrajectory(const moveit_msgs::RobotTrajectory& trajectory,
                                   bool cartesian_time_parametrization = false);

  RLLErrorCode computeLinearPathArmangle(const std::vector<geometry_msgs::Pose>& waypoints_pose,
                                         const std::vector<double>& waypoints_arm_angles,
                                         const std::vector<double>& ik_seed_state,
                                         std::vector<robot_state::RobotStatePtr>* path);
  RLLErrorCode computeLinearPath(const geometry_msgs::Pose& goal, moveit_msgs::RobotTrajectory* trajectory);
  void transformPoseForIK(geometry_msgs::Pose* pose);
  void transformPoseFromFK(geometry_msgs::Pose* pose);
  RLLErrorCode interpolatePosesLinear(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end,
                                      std::vector<geometry_msgs::Pose>* waypoints, size_t steps_arm_angle = 0);
  void interpolateArmangleLinear(double start, double end, int dir, int n, std::vector<double>* arm_angles);
  std::vector<double> getJointValuesFromNamedTarget(const std::string& name);
  bool armangleInRange(double arm_angle);
  size_t numStepsArmAngle(double start, double end);

  bool manipCurrentStateAvailable();
  robot_state::RobotState getCurrentRobotState(bool wait_for_state = false);
  void disableCollision(const std::string& link_1, const std::string& link_2);
  bool jointsGoalTooClose(const std::vector<double>& start, const std::vector<double>& goal);
  bool poseGoalTooClose(const geometry_msgs::Pose& goal);
  RLLErrorCode poseGoalInCollision(const geometry_msgs::Pose& goal, std::vector<double>* goal_joint_values);
  bool attachGraspObject(const std::string& object_id);
  bool detachGraspObject(const std::string& object_id);

  virtual bool modifyLinTrajectory(moveit_msgs::RobotTrajectory* trajectory);
  virtual RLLErrorCode closeGripper();
  virtual RLLErrorCode openGripper();

  // this method can be used to handle critical failures, e.g. set error state in the state machine
  virtual void abortDueToCriticalFailure() = 0;

  // the following methods depend on whether we run in simulation or on the real robot
  // the actual implementation is implemented in a subclass, e.g. RLLMoveIfaceSimulation
  virtual bool modifyPtpTrajectory(moveit_msgs::RobotTrajectory* trajectory) = 0;

private:
  static const std::string MANIP_PLANNING_GROUP;
  static const std::string GRIPPER_PLANNING_GROUP;
  // internal moveit named targets
  static const std::string GRIPPER_OPEN_TARGET_NAME;
  static const std::string GRIPPER_CLOSE_TARGET_NAME;

  std::string ns_;
  moveit::planning_interface::MoveGroupInterface gripper_move_group_;
  tf::Transform base_to_world_;
  tf::Transform ee_to_tip_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  collision_detection::AllowedCollisionMatrix acm_;

  double allowed_start_tolerance_ = 0.01;

  RLLErrorCode execute(moveit::planning_interface::MoveGroupInterface* move_group,
                       const moveit::planning_interface::MoveGroupInterface::Plan& plan);

  bool jointsGoalInCollision(const std::vector<double>& goal);
  RLLErrorCode checkTrajectory(const moveit_msgs::RobotTrajectory& trajectory);
  bool stateInCollision(robot_state::RobotState* state);
  float distanceToCurrentPosition(const geometry_msgs::Pose& pose);

  void getPathIK(const std::vector<geometry_msgs::Pose>& waypoints_pose, const std::vector<double>& ik_seed_state,
                 std::vector<robot_state::RobotStatePtr>* path, double* last_valid_percentage);
  void getPathIK(const std::vector<geometry_msgs::Pose>& waypoints_pose,
                 const std::vector<double>& waypoints_arm_angles, const std::vector<double>& ik_seed_state,
                 std::vector<robot_state::RobotStatePtr>* path, double* last_valid_percentage);

  bool getKinematicsSolver();
  bool initConstTransforms();  // init members base_to_world_ ee_to_tip_
  bool isCollisionLinkAvailable();
};

#endif  // RLL_MOVE_MOVE_IFACE_PLANNING_H
