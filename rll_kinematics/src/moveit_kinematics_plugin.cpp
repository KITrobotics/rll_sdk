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

#include <moveit/rdf_loader/rdf_loader.h>

#include <rll_kinematics/moveit_kinematics_plugin.h>

namespace rll_moveit_kinematics
{
#if ROS_VERSION_MINIMUM(1, 14, 3)  // Melodic
#else
static void noDeleter(const moveit::core::RobotModel* /*unused*/)
{
}
#endif

bool RLLMoveItKinematicsPlugin::initialize(
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

  rdf_loader::RDFLoader rdf_loader(robot_description_);
  const srdf::ModelSharedPtr& srdf = rdf_loader.getSRDF();
  const urdf::ModelInterfaceSharedPtr& urdf_model = rdf_loader.getURDF();

  if (!urdf_model || !srdf)
  {
    ROS_ERROR("URDF and SRDF must be loaded for kinematics solver to work.");
    return false;
  }

  static moveit::core::RobotModel robot_model_instance(urdf_model, srdf);
  robot_model_ = moveit::core::RobotModelConstPtr(&robot_model_instance, &noDeleter);
#endif

  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name);
  if (jmg == nullptr)
  {
    ROS_ERROR_STREAM("Unknown planning group: " << group_name);
    return false;
  }

  return setLimbLengthsJointLimits();
}

bool RLLMoveItKinematicsPlugin::getPositionIK(  // NOLINT google-default-arguments
    const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, std::vector<double>& solution,
    moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& /*options*/) const
{
  RLLKinPoseConfig cart_pose;
  RLLKinJoints seed_state, ik_solution;
  RLLInvKinOptions ik_options;

  cart_pose.pose.setPosition(ik_pose.position.x, ik_pose.position.y, ik_pose.position.z);
  cart_pose.pose.setQuaternion(ik_pose.orientation.w, ik_pose.orientation.x, ik_pose.orientation.y,
                               ik_pose.orientation.z);
  seed_state.setJoints(ik_seed_state);

  RLLKinMsg result = solver_.ik(seed_state, &cart_pose, &ik_solution, ik_options);
  if (result.error())
  {
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  ik_solution.getJoints(&solution);
  error_code.val = error_code.SUCCESS;

  return true;
}

bool RLLMoveItKinematicsPlugin::searchPositionIK(  // NOLINT google-default-arguments
    const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double /*timeout*/,
    std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool RLLMoveItKinematicsPlugin::searchPositionIK(  // NOLINT google-default-arguments
    const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double /*timeout*/,
    const std::vector<double>& /*consistency_limits*/, std::vector<double>& solution,
    moveit_msgs::MoveItErrorCodes& error_code, const kinematics::KinematicsQueryOptions& options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool RLLMoveItKinematicsPlugin::searchPositionIK(  // NOLINT google-default-arguments
    const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double /*timeout*/,
    std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const
{
  bool success = getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
  if (!success)
  {
    return false;
  }

  if (solution_callback.empty())
  {
    return true;
  }

  // check for collisions if a callback is provided
  solution_callback(ik_pose, solution, error_code);

  return error_code.val == error_code.SUCCESS;
}

bool RLLMoveItKinematicsPlugin::searchPositionIK(  // NOLINT google-default-arguments
    const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
    const std::vector<double>& /*consistency_limits*/, std::vector<double>& solution,
    const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, options);
}

bool RLLMoveItKinematicsPlugin::getPositionFK(const std::vector<std::string>& /* link_names */,
                                              const std::vector<double>& joint_angles,
                                              std::vector<geometry_msgs::Pose>& poses) const
{
  double arm_angle;
  int config;

  return getPositionFK(joint_angles, &poses[0], &arm_angle, &config);
}

bool RLLMoveItKinematicsPlugin::getPositionIKarmangle(const geometry_msgs::Pose& ik_pose,
                                                      const std::vector<double>& ik_seed_state,
                                                      std::vector<double>* solution,
                                                      moveit_msgs::MoveItErrorCodes* error_code,
                                                      const double& arm_angle) const
{
  RLLKinPoseConfig cart_pose;
  RLLKinJoints seed_state, ik_solution;
  RLLInvKinOptions ik_options;

  cart_pose.arm_angle = arm_angle;
  cart_pose.pose.setPosition(ik_pose.position.x, ik_pose.position.y, ik_pose.position.z);
  cart_pose.pose.setQuaternion(ik_pose.orientation.w, ik_pose.orientation.x, ik_pose.orientation.y,
                               ik_pose.orientation.z);
  seed_state.setJoints(ik_seed_state);

  ik_options.method = RLLInvKinOptions::ARM_ANGLE_FIXED;
  RLLKinMsg result = solver_.ik(seed_state, &cart_pose, &ik_solution, ik_options);
  if (result.error())
  {
    error_code->val = error_code->NO_IK_SOLUTION;
    return false;
  }

  ik_solution.getJoints(solution);
  error_code->val = error_code->SUCCESS;

  return true;
}

bool RLLMoveItKinematicsPlugin::getPositionFK(const std::vector<double>& joint_angles, geometry_msgs::Pose* pose,
                                              double* arm_angle, int* config) const
{
  RLLKinJoints ik_joint_angles;
  RLLKinPoseConfig cart_pose;

  ik_joint_angles.setJoints(joint_angles);
  RLLKinMsg result = solver_.fk(ik_joint_angles, &cart_pose);

  if (result.error())
  {
    return false;
  }

  pose->position.x = cart_pose.pose.pos()[0];
  pose->position.y = cart_pose.pose.pos()[1];
  pose->position.z = cart_pose.pose.pos()[2];
  cart_pose.pose.getQuaternion(&pose->orientation.w, &pose->orientation.x, &pose->orientation.y, &pose->orientation.z);
  *arm_angle = cart_pose.arm_angle;
  *config = cart_pose.config.val();

  return true;
}

bool RLLMoveItKinematicsPlugin::setLimbLengthsJointLimits()
{
  ROS_DEBUG_STREAM("Registering joints and links");
  const moveit::core::LinkModel* link = robot_model_->getLinkModel(tip_frames_[0]);
  const moveit::core::LinkModel* base_link = robot_model_->getLinkModel(base_frame_);
  std::vector<double> lower_joint_limits;
  std::vector<double> upper_joint_limits;

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
      lower_joint_limits.push_back(bounds.min_position_);
      upper_joint_limits.push_back(bounds.max_position_);
    }
    link = link->getParentLinkModel();
  }

  if (joint_names_.size() != RLL_NUM_JOINTS)
  {
    ROS_ERROR_STREAM("Number of joints from RobotModel (" << joint_names_.size() << ") and IK-solver ("
                                                          << RLL_NUM_JOINTS << ") do not match");

    return false;
  }

  std::reverse(link_names_.begin(), link_names_.end());
  std::reverse(joint_names_.begin(), joint_names_.end());
  std::reverse(lower_joint_limits.begin(), lower_joint_limits.end());
  std::reverse(upper_joint_limits.begin(), upper_joint_limits.end());

  ROS_DEBUG("joint limits:");
  for (size_t joint_id = 0; joint_id < RLL_NUM_JOINTS; ++joint_id)
  {
    ROS_DEBUG_STREAM(joint_names_[joint_id] << " " << lower_joint_limits[joint_id] << " "
                                            << upper_joint_limits[joint_id]);
  }

// get parent_to_joint_origin_transform.position.z for all joints from urdf to calculate limb lengths:
#if ROS_VERSION_MINIMUM(1, 14, 3)  // Melodic
  std::shared_ptr<const urdf::ModelInterface> robot_model_urdf;
  robot_model_urdf = robot_model_->getURDF();
  std::shared_ptr<const urdf::Joint> urdf_joint;
#else  // Kinetic and older
  const urdf::ModelInterfaceSharedPtr robot_model_urdf =  // NOLINT readability-identifier-naming
      robot_model_->getURDF();
  urdf::JointConstSharedPtr urdf_joint;
#endif

  std::vector<double> joint_distances;
  for (auto& joint_name : joint_names_)
  {
    urdf_joint = robot_model_urdf->getJoint(joint_name);
    joint_distances.push_back(urdf_joint->parent_to_joint_origin_transform.position.z);
  }

  // for end effector joint (fixed)
  link = robot_model_->getLinkModel(tip_frames_[0]);
  const moveit::core::JointModel* joint = link->getParentJointModel();
  urdf_joint = robot_model_urdf->getJoint(joint->getName());
  joint_distances.push_back(urdf_joint->parent_to_joint_origin_transform.position.z);
  ROS_DEBUG_STREAM(joint->getName() << " " << joint_distances[RLL_NUM_JOINTS]);

  RLLKinLimbs limb_lengths;
  limb_lengths[0] = joint_distances[0] + joint_distances[1];
  limb_lengths[1] = joint_distances[2] + joint_distances[3];
  limb_lengths[2] = joint_distances[4] + joint_distances[5];
  limb_lengths[3] = joint_distances[6] + joint_distances[7];
  RLLKinJoints rllkin_lower_joint_limits = lower_joint_limits;
  RLLKinJoints rllkin_upper_joint_limits = upper_joint_limits;

  solver_.initialize(limb_lengths, rllkin_lower_joint_limits, rllkin_upper_joint_limits);

  return true;
}

}  // namespace rll_moveit_kinematics

PLUGINLIB_EXPORT_CLASS(rll_moveit_kinematics::RLLMoveItKinematicsPlugin, kinematics::KinematicsBase)
