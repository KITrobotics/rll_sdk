/*
 * This file is part of the Robot Learning Lab SDK
 *
 * Copyright (C) 2019 Philipp Altoe <updim@student.kit.edu>
 * Copyright (C) 2020 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
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

#ifndef RLL_MOVEIT_KINEMATICS_PLUGIN_MOVEIT_KINEMATICS_PLUGIN_H
#define RLL_MOVEIT_KINEMATICS_PLUGIN_MOVEIT_KINEMATICS_PLUGIN_H

// ros
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <urdf/model.h>

// moveit
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_state/robot_state.h>

#include <rll_kinematics/redundancy_resolution.h>

namespace rll_moveit_kinematics
{
class RLLMoveItKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  RLLMoveItKinematicsPlugin() = default;

  bool getPositionIK(  // NOLINT google-default-arguments
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, std::vector<double>& solution,
      moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(  // NOLINT google-default-arguments
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(  // NOLINT google-default-arguments
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(  // NOLINT google-default-arguments
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(  // NOLINT google-default-arguments
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::Pose>& poses) const override;

#if ROS_VERSION_MINIMUM(1, 14, 3)  // Melodic
  bool initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
                  const std::string& base_frame, const std::vector<std::string>& tip_frames,
                  double search_discretization) override;
#else  // Kinetic and older
  bool initialize(const std::string& robot_description, const std::string& group_name, const std::string& base_frame,
                  const std::string& tip_frame, double search_discretization) override;
#endif

  const std::vector<std::string>& getJointNames() const override
  {
    return joint_names_;
  };

  const std::vector<std::string>& getLinkNames() const override
  {
    return link_names_;
  };

  // solve IK with a given arm angle
  bool getPositionIKarmangle(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                             std::vector<double>* solution, moveit_msgs::MoveItErrorCodes* error_code,
                             const double& arm_angle) const;
  // solve forward kinematics and get arm angle and global configuration
  bool getPositionFK(const std::vector<double>& joint_angles, geometry_msgs::Pose* pose, double* arm_angle,
                     int* config) const;

  RLLKinMsg callRLLIK(const geometry_msgs::Pose& ros_pose, const RLLKinSeedState& ik_seed_state,
                      RLLKinSolutions* solutions, RLLInvKinOptions ik_options) const;
  RLLKinMsg callRLLIK(const RLLKinSeedState& ik_seed_state, RLLKinPoseConfig* ik_pose, RLLKinSolutions* solutions,
                      RLLInvKinOptions ik_options) const;

  static void transformPose(const geometry_msgs::Pose& ros_pose, RLLKinPoseConfig* ik_pose);

private:
  bool setLimbLengthsJointLimits();

  RLLRedundancyResolution solver_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
#if ROS_VERSION_MINIMUM(1, 14, 3)  // Melodic
#else
  moveit::core::RobotModelConstPtr robot_model_;  // add member for Kinetic
#endif
};
}  // namespace rll_moveit_kinematics

#endif  // RLL_MOVEIT_KINEMATICS_PLUGIN_MOVEIT_KINEMATICS_PLUGIN_H
