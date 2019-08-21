/*
 * This file is part of the Robot Learning Lab SDK
 *
 * Copyright (C) 2019 Philipp Altoe <updim@student.kit.edu>
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

#ifndef INCLUDE_RLL_MOVEIT_ANALYTICAL_KINEMATICS_PLUGIN_H_
#define INCLUDE_RLL_MOVEIT_ANALYTICAL_KINEMATICS_PLUGIN_H_

// determine mode of inverse kinematics calculation:
// INV_KIN_MODE == 0: redundancy resolution using e-function and hard coded config in getPositionIK()
// INV_KIN_MODE == 1  solve for hard coded nsparam and config in getPositionIK()
// INV_KIN_MODE == 2  general redundancy-resolution using e-function
#define INV_KIN_MODE 2

// library includes

// InvKin
#include "inverse_kinematics_library.h"

// ros
#include <ros/ros.h>
#include <urdf/model.h>
#include <tf/transform_datatypes.h>
#include <pluginlib/class_list_macros.h>

// moveit
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_state/robot_state.h>

// debugging
//#include <iostream>

namespace rll_moveit_analytical_kinematics
{
class RLLMoveItAnalyticalKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  static bool first_call_IK_;

  RLLMoveItAnalyticalKinematicsPlugin();

  bool getPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, std::vector<double>& solution,
      moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
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
    return joint_names;
  };

  const std::vector<std::string>& getLinkNames() const override
  {
    return link_names;
  };

  bool getPositionIKelb(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                        std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                        const double elbow_angle) const;

private:
  InvKin solver;
  bool initialized;  // Indicates if parameters are initialized
  std::vector<std::string> joint_names;
  std::vector<double> joint_min_vector;
  std::vector<double> joint_max_vector;
  std::vector<double> joint_origin_z_vector;  // necessary to calculate limb lengths
  std::vector<std::string> link_names;
  const size_t num_joints = 7;  // IK for 7 DOF robots
};
}

#endif /* INCLUDE_RLL_MOVEIT_ANALYTICAL_KINEMATICS_PLUGIN_H_ */
