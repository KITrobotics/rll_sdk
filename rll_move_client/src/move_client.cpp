/*
 * This file is part of the Robot Learning Lab Move Client
 *
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

#include <rll_move_client/move_client.h>

const char* AnsiCodes::END_ = "\x1B[0m";
const char* AnsiCodes::RED_ = "\x1B[91m";
const char* AnsiCodes::GRN_ = "\x1B[92m";
const char* AnsiCodes::YEL_ = "\x1B[93m";
const char* AnsiCodes::BLU_ = "\x1B[94m";
const char* AnsiCodes::MAG_ = "\x1B[95m";
const char* AnsiCodes::CYN_ = "\x1B[96m";
const char* AnsiCodes::WHT_ = "\x1B[97m";
const char* AnsiCodes::BOLD_ = "\x1B[1m";
const char* AnsiCodes::NAME_ = "\x1B[1m\x1B[94m";
const char* AnsiCodes::OK_ = "\x1B[1m\x1B[92m";
const char* AnsiCodes::INFO_ = "\x1B[1m\x1B[93m";
const char* AnsiCodes::WARN_ = "\x1B[1m\x1B[95m";
const char* AnsiCodes::FAIL_ = "\x1B[1m\x1B[91m";

// hints are only visible within this compilation unit
const static std::map<uint8_t, const std::string> HINTS = {

  { RLLErrorCode::JOINT_VALUES_OUT_OF_RANGE,
    "One or more of the joint values you specified are outside their allowed limits." },
  { RLLErrorCode::INVALID_TARGET_POSE, "The pose/joint values you specified cannot be reached i.e. they are outside "
                                       "their allowed limits, or would move the robot outside the allowed workspace." },
  { RLLErrorCode::TOO_FEW_WAYPOINTS,
    "The distance to the requested goal pose is probably too small (e.g. less than 5mm for a linear motion)." },
  { RLLErrorCode::GOAL_TOO_CLOSE_TO_START,
    "The robot is already at/too close to the goal and no motion is performed." },
  { RLLErrorCode::NO_IK_SOLUTION_FOUND,
    "The inverse kinematics did not yield a solution. Is your goal pose within the workspace?" },
  { RLLErrorCode::NO_RANDOM_POSITION_FOUND,
    "Random pose generation may fail e.g. if the generated pose is in collision." },
  { RLLErrorCode::GOAL_IN_COLLISION,
    "The request motion would result in a collision either with an obstacle or the robot itself." },
  { RLLErrorCode::MOVEIT_PLANNING_FAILED, "This is a generic motion planning error and can be caused "
                                          "e.g. by requesting a pose outside the allowed workspace." },
  { RLLErrorCode::ONLY_PARTIAL_PATH_PLANNED, "A pose between the start and goal pose of a linear motion causes a "
                                             "collision, only part of the motion is possible." },
  { RLLErrorCode::SERVICE_CALL_NOT_ALLOWED,
    "The movement interface currently does not accept service calls, possibly due to a critical failure." },
};

void RLLMoveClientBase::logHint(RLLErrorCode error_code)
{
  auto iter = HINTS.find(error_code.value());
  if (iter != HINTS.end())
  {
    const std::string HINT = iter->second;
    ROS_INFO("%sPossible failure reason:%s %s", AnsiCodes::INFO_, AnsiCodes::END_, HINT.c_str());
  }
}

bool RLLMoveClientBase::handleResponseWithoutErrorCode(const std::string& srv_name, bool call_success)
{
  std::string name = AnsiCodes::NAME_ + srv_name + AnsiCodes::END_;
  if (!call_success)
  {
    ROS_ERROR("Failed to call '%s' service.", name.c_str());
    return false;
  }
  return true;
}

bool RLLMoveClientBase::handleResponseWithErrorCode(const std::string& srv_name, bool call_success,
                                                    RLLErrorCode error_code)
{
  std::string name = AnsiCodes::NAME_ + srv_name + AnsiCodes::END_;
  if (!call_success)
  {
    ROS_ERROR("Failed to call '%s' service.", name.c_str());
    return false;
  }

  if (error_code.succeeded())
  {
    ROS_INFO("%s %ssucceeded%s.", name.c_str(), AnsiCodes::OK_, AnsiCodes::END_);
    return true;
  }

  if (error_code.isCriticalFailure())
  {
    ROS_ERROR("%s %sfailed critically: %s%s", name.c_str(), AnsiCodes::FAIL_, error_code.message(), AnsiCodes::END_);

    throw CriticalServiceCallFailure("Service call " + srv_name + "failed critically: " + error_code.message());
  }

  ROS_WARN("%s %sfailed: %s%s", name.c_str(), AnsiCodes::FAIL_, error_code.message(), AnsiCodes::END_);

  if (exception_on_any_failure_)
  {
    throw ServiceCallFailure("Service call " + srv_name + "failed: " + error_code.message());
  }

  if (error_code.value() == RLLErrorCode::JOB_EXECUTION_TIMED_OUT)
  {
    throw ServiceCallFailure("You exceeded the maximum job execution duration.");
  }

  if (verbose_)
  {
    logHint(error_code);
  }

  return false;
}

RLLBasicMoveClient::RLLBasicMoveClient()
  : move_joints_(nh_.serviceClient<rll_msgs::MoveJoints>(RLLMoveIfaceServices::MOVE_JOINTS_SRV_NAME))
  , move_ptp_(nh_.serviceClient<rll_msgs::MovePTP>(RLLMoveIfaceServices::MOVE_PTP_SRV_NAME))
  , move_lin_(nh_.serviceClient<rll_msgs::MoveLin>(RLLMoveIfaceServices::MOVE_LIN_SRV_NAME))
  , move_ptp_armangle_(nh_.serviceClient<rll_msgs::MovePTPArmangle>(RLLMoveIfaceServices::MOVE_PTP_ARMANGLE_SRV_NAME))
  , move_lin_armangle_(nh_.serviceClient<rll_msgs::MoveLinArmangle>(RLLMoveIfaceServices::MOVE_LIN_ARMANGLE_SRV_NAME))
  , move_random_(nh_.serviceClient<rll_msgs::MoveRandom>(RLLMoveIfaceServices::MOVE_RANDOM_SRV_NAME))
{
}

bool RLLBasicMoveClient::moveRandom(geometry_msgs::Pose* const result_pose)
{
  rll_msgs::MoveRandom move_random_msg;

  bool success = callServiceWithErrorCode<rll_msgs::MoveRandom>(RLLMoveIfaceServices::MOVE_RANDOM_SRV_NAME,
                                                                move_random_, move_random_msg);
  if (success)
  {
    *result_pose = move_random_msg.response.pose;
  }
  return success;
}

bool RLLBasicMoveClient::movePTP(const geometry_msgs::Pose& pose)
{
  rll_msgs::MovePTP move_ptp_msg;
  move_ptp_msg.request.pose = pose;

  return callServiceWithErrorCode<rll_msgs::MovePTP>(RLLMoveIfaceServices::MOVE_PTP_SRV_NAME, move_ptp_, move_ptp_msg);
}

bool RLLBasicMoveClient::movePTPArmangle(const geometry_msgs::Pose& pose, double arm_angle)
{
  rll_msgs::MovePTPArmangle move_ptp_msg;
  move_ptp_msg.request.pose = pose;
  move_ptp_msg.request.arm_angle = arm_angle;

  return callServiceWithErrorCode<rll_msgs::MovePTPArmangle>(RLLMoveIfaceServices::MOVE_PTP_ARMANGLE_SRV_NAME,
                                                             move_ptp_armangle_, move_ptp_msg);
}

bool RLLBasicMoveClient::moveLin(const geometry_msgs::Pose& pose)
{
  rll_msgs::MoveLin move_lin_msg;
  move_lin_msg.request.pose = pose;

  return callServiceWithErrorCode<rll_msgs::MoveLin>(RLLMoveIfaceServices::MOVE_LIN_SRV_NAME, move_lin_, move_lin_msg);
}

bool RLLBasicMoveClient::moveLinArmangle(const geometry_msgs::Pose& pose, double arm_angle, bool direction)
{
  rll_msgs::MoveLinArmangle move_lin_msg;
  move_lin_msg.request.pose = pose;
  move_lin_msg.request.arm_angle = arm_angle;
  move_lin_msg.request.direction = static_cast<unsigned char>(direction);

  return callServiceWithErrorCode<rll_msgs::MoveLinArmangle>(RLLMoveIfaceServices::MOVE_LIN_ARMANGLE_SRV_NAME,
                                                             move_lin_armangle_, move_lin_msg);
}

bool RLLBasicMoveClient::moveJoints(const std::vector<double>& joint_values)
{
  if (joint_values.size() < RLL_NUM_JOINTS)
  {
    ROS_WARN("You need to pass seven joint values");
    return false;
  }
  return moveJoints(joint_values[0], joint_values[1], joint_values[2], joint_values[3], joint_values[4],
                    joint_values[5], joint_values[6]);
}

// NOLINTNEXTLINE readability-function-size
bool RLLBasicMoveClient::moveJoints(double a1, double a2, double a3, double a4, double a5, double a6, double a7)
{
  rll_msgs::MoveJoints move_joints_msg;
  move_joints_msg.request.joint_1 = a1;
  move_joints_msg.request.joint_2 = a2;
  move_joints_msg.request.joint_3 = a3;
  move_joints_msg.request.joint_4 = a4;
  move_joints_msg.request.joint_5 = a5;
  move_joints_msg.request.joint_6 = a6;
  move_joints_msg.request.joint_7 = a7;

  return callServiceWithErrorCode<rll_msgs::MoveJoints>(RLLMoveIfaceServices::MOVE_JOINTS_SRV_NAME, move_joints_,
                                                        move_joints_msg);
}

bool RLLGetPoseMoveClient::getCurrentPose(geometry_msgs::Pose* const pose)
{
  rll_msgs::GetPose get_pose_msg;
  bool success = callServiceWithErrorCode(RLLMoveIfaceServices::GET_POSE_SRV_NAME, get_pose_, get_pose_msg);
  if (success)
  {
    *pose = get_pose_msg.response.pose;
  }
  return success;
}

bool RLLGetPoseMoveClient::getCurrentJointValues(std::vector<double>* const joint_values)
{
  rll_msgs::GetJointValues get_joint_values_msg;
  bool success = callServiceWithErrorCode(RLLMoveIfaceServices::GET_JOINT_VALUES_SRV_NAME, get_joint_values_,
                                          get_joint_values_msg);
  if (success)
  {
    joint_values->push_back(get_joint_values_msg.response.joint_1);
    joint_values->push_back(get_joint_values_msg.response.joint_2);
    joint_values->push_back(get_joint_values_msg.response.joint_3);
    joint_values->push_back(get_joint_values_msg.response.joint_4);
    joint_values->push_back(get_joint_values_msg.response.joint_5);
    joint_values->push_back(get_joint_values_msg.response.joint_6);
    joint_values->push_back(get_joint_values_msg.response.joint_7);
  }
  return success;
}

bool RLLPickPlaceClient::pickPlace(const geometry_msgs::Pose& pose_above, const geometry_msgs::Pose& pose_grip,
                                   bool gripper_close, const std::string& grasp_object)
{
  rll_msgs::PickPlace pick_place_msg;
  pick_place_msg.request.pose_above = pose_above;
  pick_place_msg.request.pose_grip = pose_grip;
  pick_place_msg.request.gripper_close = gripper_close ? 1u : 0u;
  pick_place_msg.request.grasp_object = grasp_object;

  bool success = callServiceWithErrorCode<rll_msgs::PickPlace>(RLLMoveIfaceServices::PICK_PLACE_SRV_NAME, pick_place_,
                                                               pick_place_msg);
  return success;
}
