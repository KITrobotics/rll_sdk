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

#ifndef RLL_MOVE_IFACE_H
#define RLL_MOVE_IFACE_H

#include <ros/ros.h>

#include <std_srvs/Trigger.h>
#include <rll_msgs/JobEnvAction.h>
#include <rll_msgs/PickPlace.h>
#include <rll_msgs/MoveLin.h>
#include <rll_msgs/MoveLinElb.h>
#include <rll_msgs/MovePTP.h>
#include <rll_msgs/MovePTPelb.h>
#include <rll_msgs/MoveJoints.h>
#include <rll_msgs/MoveRandom.h>
#include <rll_msgs/GetPose.h>
#include <rll_msgs/GetJointValues.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <actionlib/server/simple_action_server.h>
#include <rll_move/move_iface_error.h>
#include <rll_move/move_iface_state_machine.h>
#include <rll_move/permissions.h>
#include <rll_analytical_kinematics/rll_moveit_analytical_kinematics_plugin.h>

class RLLMoveIface
{
public:
  explicit RLLMoveIface();
  virtual ~RLLMoveIface() = default;

  static const std::string MANIP_PLANNING_GROUP;
  static const std::string GRIPPER_PLANNING_GROUP;

  // Available services names
  static const std::string IDLE_JOB_SRV_NAME;
  static const std::string RUN_JOB_SRV_NAME;
  static const std::string ROBOT_READY_SRV_NAME;
  static const std::string MOVE_PTP_SRV_NAME;
  static const std::string MOVE_PTP_ELB_SRV_NAME;
  static const std::string MOVE_LIN_SRV_NAME;
  static const std::string MOVE_LIN_ELB_SRV_NAME;
  static const std::string MOVE_JOINTS_SRV_NAME;
  static const std::string MOVE_RANDOM_SRV_NAME;
  static const std::string PICK_PLACE_SRV_NAME;
  static const std::string GET_POSE_SRV_NAME;
  static const std::string GET_JOINT_VALUES_SRV_NAME;

  RLLErrorCode resetToHome();

  // the public interface exposes only action and service end points
  using JobServer = actionlib::SimpleActionServer<rll_msgs::JobEnvAction>;
  void runJobAction(const rll_msgs::JobEnvGoalConstPtr& goal, JobServer* as);
  void idleAction(const rll_msgs::JobEnvGoalConstPtr& goal, JobServer* as);

  bool robotReadySrv(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool pickPlaceSrv(rll_msgs::PickPlace::Request& req, rll_msgs::PickPlace::Response& resp);
  bool moveLinSrv(rll_msgs::MoveLin::Request& req, rll_msgs::MoveLin::Response& resp);
  bool moveLinElbSrv(rll_msgs::MoveLinElb::Request& req, rll_msgs::MoveLinElb::Response& resp);
  bool movePTPSrv(rll_msgs::MovePTP::Request& req, rll_msgs::MovePTP::Response& resp);
  bool movePTPelbSrv(rll_msgs::MovePTPelb::Request& req, rll_msgs::MovePTPelb::Response& resp);
  bool moveJointsSrv(rll_msgs::MoveJoints::Request& req, rll_msgs::MoveJoints::Response& resp);
  bool moveRandomSrv(rll_msgs::MoveRandom::Request& req, rll_msgs::MoveRandom::Response& resp);
  bool getCurrentPoseSrv(rll_msgs::GetPose::Request& req, rll_msgs::GetPose::Response& resp);
  bool getCurrentJointValuesSrv(rll_msgs::GetJointValues::Request& req, rll_msgs::GetJointValues::Response& resp);

protected:
  // internal moveit named targets
  static const std::string HOME_TARGET_NAME;
  static const std::string GRIPPER_OPEN_TARGET_NAME;
  static const std::string GRIPPER_CLOSE_TARGET_NAME;

  std::string ns_;
  std::string node_name_;
  RLLMoveIfaceStateMachine iface_state_;
  moveit::planning_interface::MoveGroupInterface manip_move_group_;
  moveit::planning_interface::MoveGroupInterface gripper_move_group_;
  moveit::core::RobotModelConstPtr manip_model_;
  const robot_state::JointModelGroup* manip_joint_model_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  bool no_gripper_attached_ = false;
  tf::Transform base_to_world_;
  tf::Transform ee_to_tip_;
  Permissions permissions_;
  Permissions::Index move_permission_;
  Permissions::Index pick_place_permission_;

  virtual void setupPermissions();
  bool beforeActionExecution(RLLMoveIfaceState state, rll_msgs::JobEnvResult* result);
  bool afterActionExecution(rll_msgs::JobEnvResult* result);

  virtual RLLErrorCode idle();
  virtual void runJob(const rll_msgs::JobEnvGoalConstPtr& goal, rll_msgs::JobEnvResult& result) = 0;
  virtual void abortDueToCriticalFailure();

  RLLErrorCode pickPlace(rll_msgs::PickPlace::Request& req, rll_msgs::PickPlace::Response& resp);
  RLLErrorCode moveLin(rll_msgs::MoveLin::Request& req, rll_msgs::MoveLin::Response& resp);
  RLLErrorCode moveLinElb(rll_msgs::MoveLinElb::Request& req, rll_msgs::MoveLinElb::Response& resp);
  RLLErrorCode movePTP(rll_msgs::MovePTP::Request& req, rll_msgs::MovePTP::Response& resp);
  RLLErrorCode movePTPelb(rll_msgs::MovePTPelb::Request& req, rll_msgs::MovePTPelb::Response& resp);
  RLLErrorCode moveJoints(rll_msgs::MoveJoints::Request& req, rll_msgs::MoveJoints::Response& resp);
  RLLErrorCode moveRandom(rll_msgs::MoveRandom::Request& req, rll_msgs::MoveRandom::Response& resp);

  bool jointsGoalTooClose(const std::vector<double>& start, const std::vector<double>& goal);
  bool poseGoalTooClose(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal);
  RLLErrorCode poseGoalInCollision(const geometry_msgs::Pose& goal);
  bool jointsGoalInCollision(const std::vector<double>& goal);
  robot_state::RobotState getCurrentRobotState(bool wait_for_state = false);
  void disableCollision(const std::string& link_1, const std::string& link_2);

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  collision_detection::AllowedCollisionMatrix acm_;

  virtual RLLErrorCode beforeNonMovementServiceCall(const std::string& srv_name);
  virtual RLLErrorCode afterNonMovementServiceCall(const std::string& srv_name, RLLErrorCode previous_error_code);

  virtual RLLErrorCode beforeMovementServiceCall(const std::string& srv_name);
  virtual RLLErrorCode afterMovementServiceCall(const std::string& srv_name, const RLLErrorCode& previous_error_code);

  template <class Request, class Response>
  bool controlledMovementExecution(Request& req, Response& resp, const std::string& srv_name,
                                   RLLErrorCode (RLLMoveIface::*move_func)(Request&, Response&));

  RLLErrorCode runPTPTrajectory(moveit::planning_interface::MoveGroupInterface& move_group, bool for_gripper = false);
  RLLErrorCode moveToGoalLinear(const geometry_msgs::Pose& goal, bool cartesian_time_parametrization = true);
  RLLErrorCode runLinearTrajectory(const moveit_msgs::RobotTrajectory& trajectory,
                                   bool cartesian_time_parametrization = true);
  RLLErrorCode checkTrajectory(moveit_msgs::RobotTrajectory& trajectory);
  bool attachGraspObject(const std::string& object_id);
  bool detachGraspObject(const std::string& object_id);
  bool manipCurrentStateAvailable();
  bool isCollisionLinkAvailable();
  bool stateInCollision(robot_state::RobotState& state);
  void handleFailureSeverity(const RLLErrorCode& error_code);
  std::vector<double> getJointValuesFromNamedTarget(const std::string& name);
  void interpolatePosesLinear(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end,
                              std::vector<geometry_msgs::Pose>& waypoints);
  void interpolateElbLinear(const double start, const double end, const int dir, const int n, std::vector<double>& elb);
  void transformPosesForIK(std::vector<geometry_msgs::Pose>& waypoints);
  bool
  getKinematicsSolver(std::shared_ptr<const rll_moveit_analytical_kinematics::RLLMoveItAnalyticalKinematicsPlugin>&);
  bool initConstTransforms();  // init members base_to_world_ ee_to_tip_
  float distanceToCurrentPosition(const geometry_msgs::Pose& pose);

  // these function depend on whether we run in simulation or on the real robot
  // the actual implementation is implemented in a subclass, e.g. RLLMoveIfaceSimulation
  virtual RLLErrorCode closeGripper() = 0;
  virtual RLLErrorCode openGripper() = 0;
  virtual bool modifyPtpTrajectory(moveit_msgs::RobotTrajectory& trajectory) = 0;
  virtual bool modifyLinTrajectory(moveit_msgs::RobotTrajectory& trajectory) = 0;
};

template <class BaseIface, class EnvironmentIface>
class RLLCombinedMoveIface : public BaseIface, public EnvironmentIface
{
public:
  explicit RLLCombinedMoveIface(ros::NodeHandle nh, const std::string& srv_name)
    : BaseIface(nh, srv_name), EnvironmentIface()
  {
  }
  explicit RLLCombinedMoveIface(ros::NodeHandle nh) : BaseIface(nh), EnvironmentIface()
  {
  }
};

/**
 * \brief Waits for the `move_group` action to become available.
 */
bool waitForMoveGroupAction();

#endif  // RLL_MOVE_IFACE_H

/*
 * Local Variables:
 * c-file-style: "google"
 * End:
 */
