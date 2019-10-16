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

#include "rll_analytical_kinematics/rll_moveit_analytical_kinematics_plugin.h"
#include <moveit/rdf_loader/rdf_loader.h>

namespace rll_moveit_analytical_kinematics
{
RLLMoveItAnalyticalKinematicsPlugin::RLLMoveItAnalyticalKinematicsPlugin() : initialized_(false)
{
}

bool RLLMoveItAnalyticalKinematicsPlugin::initialize(
#if ROS_VERSION_MINIMUM(1, 14, 3)  // Melodic
    const moveit::core::RobotModel& robot_model,
#else  // Kinetic and older
    const std::string& robot_description,
#endif
    const std::string& group_name, const std::string& base_frame,
#if ROS_VERSION_MINIMUM(1, 14, 3)  // Melodic
    const std::vector<std::string>& tip_frames,
#else  // Kinetic and older
    const std::string& tip_frame,
#endif
    double search_discretization)
{
#if ROS_VERSION_MINIMUM(1, 14, 3)  // Melodic
  if (tip_frames.size() != 1)
  {
    ROS_ERROR_STREAM("Expecting exactly one tip frame.");
    return false;
  }
#endif

#if ROS_VERSION_MINIMUM(1, 14, 3)  // Melodic
  ROS_DEBUG_STREAM("robot_model_getName()=" << robot_model.getName() << " group_name=" << group_name << " base_frame="
                                            << base_frame << " tip_frames=" << tip_frames[0]  // only one tip_frame
                                            << " search_discretization=" << search_discretization);

  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
#else  // Kinetic and older
  ROS_DEBUG_STREAM("robot_description=" << robot_description << " group_name=" << group_name
                                        << " base_frame=" << base_frame << " tip_frames=" << tip_frame
                                        << " search_discretization=" << search_discretization);

  setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);

  rdf_loader::RDFLoader rdf_loader(robot_description);
  const srdf::ModelSharedPtr& srdf = rdf_loader.getSRDF();
  const urdf::ModelInterfaceSharedPtr& urdf_model = rdf_loader.getURDF();

  if (!urdf_model || !srdf)
  {
    ROS_ERROR("URDF and SRDF must be loaded for kinematics solver to work.");
    return false;
  }

  moveit::core::RobotModel robot_model_instance_(urdf_model, srdf);
  robot_model_ = moveit::core::RobotModelConstPtr(&robot_model_instance_);
#endif

  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name);
  if (jmg == nullptr)
  {
    ROS_ERROR_STREAM("Unknown planning group: " << group_name);
    return false;
  }

  ROS_DEBUG_STREAM("Registering joints and links");
  const moveit::core::LinkModel* link = robot_model_->getLinkModel(tip_frames_[0]);
  const moveit::core::LinkModel* base_link = robot_model_->getLinkModel(base_frame_);
  while (link != nullptr && link != base_link)
  {
    ROS_DEBUG_STREAM("Link " << link->getName());
    link_names_.push_back(link->getName());
    const moveit::core::JointModel* joint = link->getParentJointModel();
    if (joint->getType() != joint->UNKNOWN && joint->getType() != joint->FIXED && joint->getVariableCount() == 1)
    {
      ROS_DEBUG_STREAM("Adding joint " << joint->getName());

      joint_names_.push_back(joint->getName());
      const moveit::core::VariableBounds& bounds = joint->getVariableBounds()[0];
      joint_min_vector_.push_back(bounds.min_position_);
      joint_max_vector_.push_back(bounds.max_position_);
    }
    link = link->getParentLinkModel();
  }

  if (joint_names_.size() != NUM_JOINTS)
  {
    ROS_ERROR_STREAM("Joint numbers of RobotModel (" << joint_names_.size() << ") and IK-solver (" << NUM_JOINTS
                                                     << ") do not match");

    return false;
  }

  std::reverse(link_names_.begin(), link_names_.end());
  std::reverse(joint_names_.begin(), joint_names_.end());
  std::reverse(joint_min_vector_.begin(), joint_min_vector_.end());
  std::reverse(joint_max_vector_.begin(), joint_max_vector_.end());

  ROS_DEBUG("joint limits:");
  for (size_t joint_id = 0; joint_id < NUM_JOINTS; ++joint_id)
  {
    ROS_DEBUG_STREAM(joint_names_[joint_id] << " " << joint_min_vector_[joint_id] << " "
                                            << joint_max_vector_[joint_id]);
  }

// get parent_to_joint_origin_transform.position.z for all joints from urdf to calculate limb lengths:
#if ROS_VERSION_MINIMUM(1, 14, 3)  // Melodic
  std::shared_ptr<const urdf::ModelInterface> robot_model_urdf;
  robot_model_urdf = robot_model.getURDF();
  std::shared_ptr<const urdf::Joint> urdf_joint;
#else  // Kinetic and older
  const urdf::ModelInterfaceSharedPtr robot_model_urdf = robot_model_instance_.getURDF();
  urdf::JointConstSharedPtr urdf_joint;
#endif

  for (auto& joint_name : joint_names_)
  {
    urdf_joint = robot_model_urdf->getJoint(joint_name);
    joint_origin_z_vector_.push_back(urdf_joint->parent_to_joint_origin_transform.position.z);
  }

  // for end effector joint (fixed)
  link = robot_model_->getLinkModel(tip_frames_[0]);
  const moveit::core::JointModel* joint = link->getParentJointModel();
  urdf_joint = robot_model_urdf->getJoint(joint->getName());
  joint_origin_z_vector_.push_back(urdf_joint->parent_to_joint_origin_transform.position.z);
  ROS_DEBUG_STREAM(joint->getName() << " " << joint_origin_z_vector_[NUM_JOINTS]);

  // initialize inverse kinematics solver
  solver_.initialize(joint_origin_z_vector_, joint_min_vector_, joint_max_vector_);

  initialized_ = true;

  return true;
}

bool RLLMoveItAnalyticalKinematicsPlugin::getPositionIK(const geometry_msgs::Pose& ik_pose,
                                                        const std::vector<double>& ik_seed_state,
                                                        std::vector<double>& solution,
                                                        moveit_msgs::MoveItErrorCodes& error_code,
                                                        const kinematics::KinematicsQueryOptions& /*options*/) const
{
  ROS_DEBUG_STREAM("getPositionIK");

  // conversion in types of inverse_kinematics_library:
  InvKinXCart cart_pose;
  cart_pose.pose.setPosition(ik_pose.position.x, ik_pose.position.y, ik_pose.position.z);
  cart_pose.pose.setQuaternion(ik_pose.orientation.w, ik_pose.orientation.x, ik_pose.orientation.y,
                               ik_pose.orientation.z);
  InvKinJoints seed_state;
  seed_state.setJoints(ik_seed_state);

  InvKinMsg kinematics_return;

// calculation depending on INV_KIN_MODE
#if INV_KIN_MODE == 0    // redundancy resolution using e-function and hard coded config
  cart_pose.config = 2;  // set fixed config
  kinematics_return = solver_.getIKefuncFixedConfig(cart_pose, seed_state, solution);
#endif  // INV_KIN_MODE==0

#if INV_KIN_MODE == 1                    // solve for hard coded nsparam and config
  cart_pose.config = 2;                  // set fixed config
  cart_pose.nsparam = 0.0 * M_PI / 2.0;  // set fixed nsparam
  kinematics_return = solver_.getIKefuncFixedConfigFixedNs(cart_pose, seed_state, solution);
#endif  // INV_KIN_MODE==1

#if INV_KIN_MODE == 2  // redundancy-resolution using e-function
  kinematics_return = solver_.getIKefunc(cart_pose, seed_state, &solution);
#endif  // INV_KIN_MODE==2

  // convert error-types
  if (kinematics_return != INVKIN_OK && kinematics_return != (INVKIN_WARNING | INVKIN_CLOSE_TO_SINGULARITY))
  {
    ROS_DEBUG_STREAM("Inverse kinematics failed: " << kinematics_return);
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  error_code.val = error_code.SUCCESS;

  return true;
}

bool RLLMoveItAnalyticalKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                           const std::vector<double>& ik_seed_state, double /*timeout*/,
                                                           std::vector<double>& solution,
                                                           moveit_msgs::MoveItErrorCodes& error_code,
                                                           const kinematics::KinematicsQueryOptions& options) const
{
  ROS_DEBUG_STREAM("searchPositionIK 1");

  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool RLLMoveItAnalyticalKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                           const std::vector<double>& ik_seed_state, double /*timeout*/,
                                                           const std::vector<double>& /*consistency_limits*/,
                                                           std::vector<double>& solution,
                                                           moveit_msgs::MoveItErrorCodes& error_code,
                                                           const kinematics::KinematicsQueryOptions& options) const
{
  ROS_DEBUG_STREAM("searchPositionIK 2");

  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool RLLMoveItAnalyticalKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                           const std::vector<double>& ik_seed_state, double /*timeout*/,
                                                           std::vector<double>& solution,
                                                           const IKCallbackFn& solution_callback,
                                                           moveit_msgs::MoveItErrorCodes& error_code,
                                                           const kinematics::KinematicsQueryOptions& options) const
{
  ROS_DEBUG_STREAM("searchPositionIK 3");

  if (getPositionIK(ik_pose, ik_seed_state, solution, error_code, options))
  {
    // check for collisions if a callback is provided
    if (!solution_callback.empty())
    {
      solution_callback(ik_pose, solution, error_code);
      if (error_code.val == error_code.SUCCESS)
      {
        ROS_DEBUG_STREAM("Solution passes callback");
        return true;
      }

      ROS_DEBUG_STREAM("Solution collides");
      return false;
    }
  }
  else
  {
    return false;
  }

  return false;
}

bool RLLMoveItAnalyticalKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                           const std::vector<double>& ik_seed_state, double /*timeout*/,
                                                           const std::vector<double>& /*consistency_limits*/,
                                                           std::vector<double>& solution,
                                                           const IKCallbackFn& solution_callback,
                                                           moveit_msgs::MoveItErrorCodes& error_code,
                                                           const kinematics::KinematicsQueryOptions& options) const
{
  ROS_DEBUG_STREAM("searchPositionIK 4");

  if (getPositionIK(ik_pose, ik_seed_state, solution, error_code, options))
  {
    // check for collisions if a callback is provided
    if (!solution_callback.empty())
    {
      solution_callback(ik_pose, solution, error_code);
      if (error_code.val == error_code.SUCCESS)
      {
        ROS_DEBUG_STREAM("Solution passes callback");
        return true;
      }

      ROS_DEBUG("Solution collides");
      return false;
    }
  }
  else
  {
    return false;
  }

  return false;
}

bool RLLMoveItAnalyticalKinematicsPlugin::getPositionFK(const std::vector<std::string>& /*link_names*/,
                                                        const std::vector<double>& joint_angles,
                                                        std::vector<geometry_msgs::Pose>& poses) const
{
  ROS_DEBUG_STREAM("getPositionFK");

  InvKinJoints angles;
  angles.setJoints(joint_angles);

  InvKinXCart cart_pose;

  InvKinMsg kinematics_return;
  kinematics_return = solver_.forwardKinematics(&cart_pose, angles);

  if (kinematics_return == INVKIN_OK)
  {
    poses.resize(1);

    poses[0].position.x = cart_pose.pose.pos[0];
    poses[0].position.y = cart_pose.pose.pos[1];
    poses[0].position.z = cart_pose.pose.pos[2];

    cart_pose.pose.getQuaternion(&poses[0].orientation.w, &poses[0].orientation.x, &poses[0].orientation.y,
                                 &poses[0].orientation.z);

    return true;
  }

  return false;
}

bool RLLMoveItAnalyticalKinematicsPlugin::getPositionIKelb(const geometry_msgs::Pose& ik_pose,
                                                           const std::vector<double>& ik_seed_state,
                                                           std::vector<double>& solution,
                                                           moveit_msgs::MoveItErrorCodes& error_code,
                                                           const double elbow_angle) const
{
  InvKinXCart cart_pose;
  cart_pose.nsparam = elbow_angle;
  cart_pose.pose.setPosition(ik_pose.position.x, ik_pose.position.y, ik_pose.position.z);
  cart_pose.pose.setQuaternion(ik_pose.orientation.w, ik_pose.orientation.x, ik_pose.orientation.y,
                               ik_pose.orientation.z);

  InvKinJoints joints;
  joints.setJoints(ik_seed_state);
  InvKinMsg kinematicsReturn;

  kinematicsReturn = solver_.getIKfixedNs(cart_pose, joints, &solution);
  if (kinematicsReturn != INVKIN_OK && kinematicsReturn != (INVKIN_WARNING | INVKIN_CLOSE_TO_SINGULARITY))
  {
    ROS_DEBUG_STREAM("inverseKinematics() failed: " << kinematicsReturn);
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  error_code.val = error_code.SUCCESS;

  return true;
}

bool RLLMoveItAnalyticalKinematicsPlugin::getPositionFKelb(const std::vector<std::string>& /*link_names*/,
                                                           const std::vector<double>& joint_angles,
                                                           std::vector<geometry_msgs::Pose>& poses,
                                                           double& elbow_angle) const
{
  InvKinJoints angles;
  angles.setJoints(joint_angles);

  InvKinXCart cart_pose;

  InvKinMsg kinematics_return;
  kinematics_return = solver_.forwardKinematics(&cart_pose, angles);

  if (kinematics_return == INVKIN_OK)
  {
    poses.resize(1);

    poses[0].position.x = cart_pose.pose.pos[0];
    poses[0].position.y = cart_pose.pose.pos[1];
    poses[0].position.z = cart_pose.pose.pos[2];

    cart_pose.pose.getQuaternion(&poses[0].orientation.w, &poses[0].orientation.x, &poses[0].orientation.y,
                                 &poses[0].orientation.z);

    elbow_angle = cart_pose.nsparam;

    return true;
  }

  return false;
}

bool RLLMoveItAnalyticalKinematicsPlugin::getPathIKelb(const std::vector<geometry_msgs::Pose>& waypoints_pose,
                                                       const std::vector<double>& waypoints_elb,
                                                       const std::vector<double>& ik_seed_state,
                                                       std::vector<robot_state::RobotStatePtr>& path,
                                                       const moveit::core::JointModelGroup* group,
                                                       moveit_msgs::MoveItErrorCodes& error_code,
                                                       double& last_valid_percentage) const
{
  // TODO(updim): test that vector pose and elb got same size
  robot_state::RobotState tmp_state(robot_model_);
  std::vector<double> sol(7);
  std::vector<double> seed_tmp = ik_seed_state;
  last_valid_percentage = 0.0;

  for (int i = 0; i < waypoints_pose.size(); i++)
  {
    if (getPositionIKelb(waypoints_pose[i], seed_tmp, sol, error_code, waypoints_elb[i]))
    {
      seed_tmp = sol;
      tmp_state.setJointGroupPositions(group, sol);
      path.push_back(robot_state::RobotStatePtr(new robot_state::RobotState(tmp_state)));  // NOLINT
    }
    else
    {
      error_code.val = error_code.NO_IK_SOLUTION;
      return false;
    }

    last_valid_percentage = static_cast<double>(i) / static_cast<double>(waypoints_pose.size() - 1);
  }

  error_code.val = error_code.SUCCESS;
  return true;
}

}  // namespace rll_moveit_analytical_kinematics

PLUGINLIB_EXPORT_CLASS(rll_moveit_analytical_kinematics::RLLMoveItAnalyticalKinematicsPlugin,
                       kinematics::KinematicsBase);
