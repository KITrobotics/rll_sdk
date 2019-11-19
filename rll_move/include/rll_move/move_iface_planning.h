/*
 * This file is part of the Robot Learning Lab SDK
 *
 * Copyright (C) 2018-2019 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
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

#ifndef RLL_MOVE_IFACE_PLANNING_H
#define RLL_MOVE_IFACE_PLANNING_H

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <rll_analytical_kinematics/rll_moveit_analytical_kinematics_plugin.h>
#include <rll_move/move_iface_error.h>

class RLLMoveIfacePlanning
{
public:
  explicit RLLMoveIfacePlanning();
  virtual ~RLLMoveIfacePlanning() = default;

  static const std::string MANIP_PLANNING_GROUP;
  static const std::string GRIPPER_PLANNING_GROUP;

protected:
  // internal moveit named targets
  static const std::string HOME_TARGET_NAME;
  static const std::string GRIPPER_OPEN_TARGET_NAME;
  static const std::string GRIPPER_CLOSE_TARGET_NAME;

  static const double DEFAULT_VELOCITY_SCALING_FACTOR;
  static const double DEFAULT_ACCELERATION_SCALING_FACTOR;
  const double DEFAULT_LINEAR_EEF_STEP = 0.0005;
  const double DEFAULT_LINEAR_JUMP_THRESHOLD = 4.5;
  const double LINEAR_MIN_STEPS_FOR_JUMP_THRESH = 10;

  std::string ns_;
  std::string node_name_;
  moveit::planning_interface::MoveGroupInterface manip_move_group_;
  moveit::planning_interface::MoveGroupInterface gripper_move_group_;
  moveit::core::RobotModelConstPtr manip_model_;
  const robot_state::JointModelGroup* manip_joint_model_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  bool no_gripper_attached_ = false;
  tf::Transform base_to_world_;
  tf::Transform ee_to_tip_;

  bool jointsGoalTooClose(const std::vector<double>& start, const std::vector<double>& goal);
  bool poseGoalTooClose(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal);
  RLLErrorCode poseGoalInCollision(const geometry_msgs::Pose& goal);
  bool jointsGoalInCollision(const std::vector<double>& goal);
  robot_state::RobotState getCurrentRobotState(bool wait_for_state = false);
  void disableCollision(const std::string& link_1, const std::string& link_2);

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  collision_detection::AllowedCollisionMatrix acm_;

  RLLErrorCode checkTrajectory(moveit_msgs::RobotTrajectory& trajectory);
  bool attachGraspObject(const std::string& object_id);
  bool detachGraspObject(const std::string& object_id);
  bool manipCurrentStateAvailable();
  bool isCollisionLinkAvailable();
  bool stateInCollision(robot_state::RobotState& state);

  void interpolatePosesLinear(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end,
                              std::vector<geometry_msgs::Pose>& waypoints);
  void interpolateElbLinear(double start, double end, int dir, int n, std::vector<double>& elb);
  void transformPosesForIK(std::vector<geometry_msgs::Pose>& waypoints);
  std::vector<double> getJointValuesFromNamedTarget(const std::string& name);
  bool getKinematicsSolver(
      std::shared_ptr<
          const rll_moveit_analytical_kinematics::RLLMoveItAnalyticalKinematicsPlugin>& /*kinematics_plugin*/);
  bool initConstTransforms();  // init members base_to_world_ ee_to_tip_
  float distanceToCurrentPosition(const geometry_msgs::Pose& pose);

  RLLErrorCode runPTPTrajectory(moveit::planning_interface::MoveGroupInterface& move_group, bool for_gripper = false);
  RLLErrorCode moveToGoalLinear(const geometry_msgs::Pose& goal, bool cartesian_time_parametrization = false);
  RLLErrorCode computeLinearPath(const std::vector<geometry_msgs::Pose>& waypoints,
                                 moveit_msgs::RobotTrajectory& trajectory);
  RLLErrorCode runLinearTrajectory(const moveit_msgs::RobotTrajectory& trajectory,
                                   bool cartesian_time_parametrization = false);

  virtual bool modifyLinTrajectory(moveit_msgs::RobotTrajectory& trajectory);
  virtual RLLErrorCode closeGripper();
  virtual RLLErrorCode openGripper();

  // this method can be used to handle critical failures, e.g. set error state in the state machine
  virtual void abortDueToCriticalFailure() = 0;

  // the following methods depend on whether we run in simulation or on the real robot
  // the actual implementation is implemented in a subclass, e.g. RLLMoveIfaceSimulation
  virtual bool modifyPtpTrajectory(moveit_msgs::RobotTrajectory& trajectory) = 0;
};

#endif  // RLL_MOVE_IFACE_PLANNING_H
