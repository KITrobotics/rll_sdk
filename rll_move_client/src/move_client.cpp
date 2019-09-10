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

const char* AnsiCodes::END = "\x1B[0m";
const char* AnsiCodes::RED = "\x1B[31m";
const char* AnsiCodes::GRN = "\x1B[32m";
const char* AnsiCodes::YEL = "\x1B[33m";
const char* AnsiCodes::BLU = "\x1B[34m";
const char* AnsiCodes::MAG = "\x1B[35m";
const char* AnsiCodes::CYN = "\x1B[36m";
const char* AnsiCodes::WHT = "\x1B[37m";
const char* AnsiCodes::BOLD = "\x1B[1m";
const char* AnsiCodes::NAME = "\x1B[1m\x1B[34m";
const char* AnsiCodes::OK = "\x1B[1m\x1B[32m";
const char* AnsiCodes::WARN = "\x1B[1m\x1B[33m";
const char* AnsiCodes::FAIL = "\x1B[1m\x1B[31m";

bool RLLMoveClientBase::handleResponseWithoutErrorCode(const std::string& srv_name, bool call_success)
{
  std::string name = AnsiCodes::NAME + srv_name + AnsiCodes::END;
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
  std::string name = AnsiCodes::NAME + srv_name + AnsiCodes::END;
  if (!call_success)
  {
    ROS_ERROR("Failed to call '%s' service.", name.c_str());
    return false;
  }

  if (error_code.succeeded())
  {
    ROS_INFO("%s %ssucceeded%s.", name.c_str(), AnsiCodes::OK, AnsiCodes::END);
    return true;
  }

  if (error_code.isCriticalFailure())
  {
    ROS_ERROR("%s failed critically: %s%s%s", name.c_str(), AnsiCodes::FAIL, error_code.message(), AnsiCodes::END);

    throw CriticalServiceCallFailure("Service call " + srv_name + "failed critically: " + error_code.message());
  }
  else
  {
    ROS_ERROR("%s failed: %s%s%s", name.c_str(), AnsiCodes::FAIL, error_code.message(), AnsiCodes::END);

    if (exception_on_any_failure_)
    {
      throw ServiceCallFailure("Service call " + srv_name + "failed: " + error_code.message());
    }
  }

  return false;
}

RLLBasicMoveClient::RLLBasicMoveClient()
  : RLLMoveClientBase()
  , move_joints_(nh_.serviceClient<rll_msgs::MoveJoints>(RLLMoveIface::MOVE_JOINTS_SRV_NAME))
  , move_ptp_(nh_.serviceClient<rll_msgs::MovePTP>(RLLMoveIface::MOVE_PTP_SRV_NAME))
  , move_lin_(nh_.serviceClient<rll_msgs::MoveLin>(RLLMoveIface::MOVE_LIN_SRV_NAME))
  , move_random_(nh_.serviceClient<rll_msgs::MoveRandom>(RLLMoveIface::MOVE_RANDOM_SRV_NAME))
{
}

bool RLLBasicMoveClient::moveRandom(geometry_msgs::Pose* const result_pose)
{
  rll_msgs::MoveRandom move_random_msg;

  bool success =
      callServiceWithErrorCode<rll_msgs::MoveRandom>(RLLMoveIface::MOVE_RANDOM_SRV_NAME, move_random_, move_random_msg);
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

  return callServiceWithErrorCode<rll_msgs::MovePTP>(RLLMoveIface::MOVE_PTP_SRV_NAME, move_ptp_, move_ptp_msg);
}

bool RLLBasicMoveClient::moveLin(const geometry_msgs::Pose& pose)
{
  rll_msgs::MoveLin move_lin_msg;
  move_lin_msg.request.pose = pose;

  return callServiceWithErrorCode<rll_msgs::MoveLin>(RLLMoveIface::MOVE_LIN_SRV_NAME, move_lin_, move_lin_msg);
}

bool RLLBasicMoveClient::moveJoints(const std::vector<double>& joint_values)
{
  if (joint_values.size() < 7)
  {
    ROS_WARN("You need to pass seven joint values");
    return false;
  }
  return moveJoints(joint_values[0], joint_values[1], joint_values[2], joint_values[3], joint_values[4],
                    joint_values[5], joint_values[6]);
}
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

  return callServiceWithErrorCode<rll_msgs::MoveJoints>(RLLMoveIface::MOVE_JOINTS_SRV_NAME, move_joints_,
                                                        move_joints_msg);
}

bool RLLGetPoseMoveClient::getCurrentPose(geometry_msgs::Pose* const pose)
{
  rll_msgs::GetPose get_pose_msg;
  bool success = callServiceWithoutErrorCode(RLLMoveIface::GET_POSE_SRV_NAME, get_pose_, get_pose_msg);
  if (success)
  {
    *pose = get_pose_msg.response.pose;
  }
  return success;
}

bool RLLGetPoseMoveClient::getCurrentJointValues(std::vector<double>* const joint_values)
{
  rll_msgs::GetJointValues get_joint_values_msg;
  bool success =
      callServiceWithoutErrorCode(RLLMoveIface::GET_JOINT_VALUES_SRV_NAME, get_joint_values_, get_joint_values_msg);
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

  bool success =
      callServiceWithErrorCode<rll_msgs::PickPlace>(RLLMoveIface::PICK_PLACE_SRV_NAME, pick_place_, pick_place_msg);
  return success;
}
