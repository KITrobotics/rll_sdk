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

#include "rll_moveit_analytical_kinematics_plugin.h"
#include <moveit/rdf_loader/rdf_loader.h>

namespace rll_moveit_analytical_kinematics
{
bool RLLMoveItAnalyticalKinematicsPlugin::first_call_IK_ = true;

RLLMoveItAnalyticalKinematicsPlugin::RLLMoveItAnalyticalKinematicsPlugin() : initialized(false)
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
  ROS_INFO_STREAM("robot_model_getName()=" << robot_model.getName() << " group_name=" << group_name << " base_frame="
                                           << base_frame << " tip_frames=" << tip_frames[0]  // only one tip_frame
                                           << " search_discretization=" << search_discretization);

  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
#else  // Kinetic and older
  ROS_INFO_STREAM("robot_description=" << robot_description << " group_name=" << group_name << " base_frame="
                                       << base_frame << " tip_frames=" << tip_frame
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
  moveit::core::RobotModel* robot_model_ = &robot_model_instance_;
#endif

  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name);
  if (!jmg)
  {
    ROS_ERROR_STREAM("Unknown planning group: " << group_name);
    return false;
  }

  ROS_INFO_STREAM("Registering joints and links");
  const moveit::core::LinkModel* link = robot_model_->getLinkModel(tip_frames_[0]);
  const moveit::core::LinkModel* base_link = robot_model_->getLinkModel(base_frame_);
  while (link && link != base_link)
  {
    ROS_INFO_STREAM("Link " << link->getName());
    link_names.push_back(link->getName());
    const moveit::core::JointModel* joint = link->getParentJointModel();
    if (joint->getType() != joint->UNKNOWN && joint->getType() != joint->FIXED && joint->getVariableCount() == 1)
    {
      ROS_INFO_STREAM("Adding joint " << joint->getName());

      joint_names.push_back(joint->getName());
      const moveit::core::VariableBounds& bounds = joint->getVariableBounds()[0];
      joint_min_vector.push_back(bounds.min_position_);
      joint_max_vector.push_back(bounds.max_position_);
    }
    link = link->getParentLinkModel();
  }

  if (joint_names.size() != num_joints)
  {
    ROS_ERROR_STREAM("Joint numbers of RobotModel (" << joint_names.size() << ") and IK-solver (" << num_joints
                                                     << ") do not match");

    return false;
  }

  std::reverse(link_names.begin(), link_names.end());
  std::reverse(joint_names.begin(), joint_names.end());
  std::reverse(joint_min_vector.begin(), joint_min_vector.end());
  std::reverse(joint_max_vector.begin(), joint_max_vector.end());

  ROS_INFO("joint limits:");
  for (size_t joint_id = 0; joint_id < num_joints; ++joint_id)
    ROS_INFO_STREAM(joint_names[joint_id] << " " << joint_min_vector[joint_id] << " " << joint_max_vector[joint_id]);

  // get parent_to_joint_origin_transform.position.z for all joints from urdf to calculate limb lengths:
#if ROS_VERSION_MINIMUM(1, 14, 3)  // Melodic
  std::shared_ptr<const urdf::ModelInterface> robot_model_urdf;
  robot_model_urdf = robot_model.getURDF();
  std::shared_ptr<const urdf::Joint> urdf_joint;
#else  // Kinetic and older
  const urdf::ModelInterfaceSharedPtr robot_model_urdf = robot_model_instance_.getURDF();
  urdf::JointConstSharedPtr urdf_joint;
#endif

  for (auto & joint_name : joint_names)
  {
    urdf_joint = robot_model_urdf->getJoint(joint_name);
    joint_origin_z_vector.push_back(urdf_joint->parent_to_joint_origin_transform.position.z);
  }

  // for end effector joint (fixed)
  link = robot_model_->getLinkModel(tip_frames_[0]);
  const moveit::core::JointModel* joint = link->getParentJointModel();
  urdf_joint = robot_model_urdf->getJoint(joint->getName());
  joint_origin_z_vector.push_back(urdf_joint->parent_to_joint_origin_transform.position.z);
  ROS_INFO_STREAM(joint->getName() << " " << joint_origin_z_vector[num_joints]);

  // initialize inverse kinematics solver
  solver.initialize(joint_origin_z_vector, joint_min_vector, joint_max_vector);

  initialized = true;

  return true;
}

bool RLLMoveItAnalyticalKinematicsPlugin::getPositionIK(const geometry_msgs::Pose& ik_pose,
                                                        const std::vector<double>& ik_seed_state,
                                                        std::vector<double>& solution,
                                                        moveit_msgs::MoveItErrorCodes& error_code,
                                                        const kinematics::KinematicsQueryOptions& options) const
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
  kinematics_return = solver.getIKefuncFixedConfig(cart_pose, seed_state, solution);
#endif  // INV_KIN_MODE==0

#if INV_KIN_MODE == 1                    // solve for hard coded nsparam and config
  cart_pose.config = 2;                  // set fixed config
  cart_pose.nsparam = 0.0 * M_PI / 2.0;  // set fixed nsparam
  kinematics_return = solver.getIKefuncFixedConfigFixedNs(cart_pose, seed_state, solution);
#endif  // INV_KIN_MODE==1

#if INV_KIN_MODE == 2  // redundancy-resolution using e-function
  kinematics_return = solver.getIKefunc(cart_pose, seed_state, solution);
#endif  // INV_KIN_MODE==2

  // convert error-types
  if (kinematics_return != InvKin_OK && kinematics_return != (InvKin_WARNING | InvKin_CLOSE_TO_SINGULARITY))
  {
    ROS_DEBUG_STREAM("Inverse kinematics failed: " << kinematics_return);
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  error_code.val = error_code.SUCCESS;

  first_call_IK_ = false;
  return true;
}

bool RLLMoveItAnalyticalKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                           const std::vector<double>& ik_seed_state, double timeout,
                                                           std::vector<double>& solution,
                                                           moveit_msgs::MoveItErrorCodes& error_code,
                                                           const kinematics::KinematicsQueryOptions& options) const
{
  ROS_DEBUG_STREAM("searchPositionIK 1");

  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool RLLMoveItAnalyticalKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                           const std::vector<double>& ik_seed_state, double timeout,
                                                           const std::vector<double>& consistency_limits,
                                                           std::vector<double>& solution,
                                                           moveit_msgs::MoveItErrorCodes& error_code,
                                                           const kinematics::KinematicsQueryOptions& options) const
{
  ROS_DEBUG_STREAM("searchPositionIK 2");

  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool RLLMoveItAnalyticalKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                           const std::vector<double>& ik_seed_state, double timeout,
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

bool RLLMoveItAnalyticalKinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
    const std::vector<double>& consistency_limits, std::vector<double>& solution, const IKCallbackFn& solution_callback,
    moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options) const
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

bool RLLMoveItAnalyticalKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                                        const std::vector<double>& joint_angles,
                                                        std::vector<geometry_msgs::Pose>& poses) const
{
  ROS_DEBUG_STREAM("getPositionFK");

  InvKinJoints angles;
  angles.setJoints(joint_angles);

  InvKinXCart cart_pose;

  InvKinMsg kinematics_return;
  kinematics_return = solver.forwardKinematics(cart_pose, angles);

  if (kinematics_return == InvKin_OK)
  {
    poses.resize(1);

    poses[0].position.x = cart_pose.pose.pos[0];
    poses[0].position.y = cart_pose.pose.pos[1];
    poses[0].position.z = cart_pose.pose.pos[2];

    cart_pose.pose.getQuaternion(poses[0].orientation.w, poses[0].orientation.x, poses[0].orientation.y,
                                 poses[0].orientation.z);

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
  cart_pose.config = 2;
  cart_pose.nsparam = elbow_angle;
  cart_pose.pose.setPosition(ik_pose.position.x, ik_pose.position.y, ik_pose.position.z);
  cart_pose.pose.setQuaternion(ik_pose.orientation.w, ik_pose.orientation.x, ik_pose.orientation.y,
                               ik_pose.orientation.z);

  InvKinJoints joints;
  InvKinMsg kinematicsReturn;

  kinematicsReturn = solver.inverseKinematics(joints, cart_pose);

  if (kinematicsReturn != InvKin_OK && kinematicsReturn != (InvKin_WARNING | InvKin_CLOSE_TO_SINGULARITY))
  {
    ROS_DEBUG_STREAM("inverseKinematics() failed: " << kinematicsReturn);
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  solution = std::vector<double>(&joints.j[0], &joints.j[0] + NR_JOINTS);
  error_code.val = error_code.SUCCESS;

  first_call_IK_ = false;

  return true;
}
}  // namespace rll_moveit_analytical_kinematics

PLUGINLIB_EXPORT_CLASS(rll_moveit_analytical_kinematics::RLLMoveItAnalyticalKinematicsPlugin,
                       kinematics::KinematicsBase);
