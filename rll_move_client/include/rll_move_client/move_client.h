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

#ifndef RLL_MOVE_CLIENT_H_
#define RLL_MOVE_CLIENT_H_

#include <exception>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <rll_msgs/MoveJoints.h>
#include <rll_msgs/MovePTP.h>
#include <rll_msgs/MoveLin.h>
#include <rll_msgs/MoveRandom.h>
#include <rll_msgs/GetJointValues.h>
#include <rll_msgs/GetPose.h>
#include <rll_msgs/PickPlace.h>

#include <rll_move/move_iface.h>

// scoped globals
struct AnsiCodes
{
  static const char* END;
  static const char* RED;
  static const char* GRN;
  static const char* YEL;
  static const char* BLU;
  static const char* MAG;
  static const char* CYN;
  static const char* WHT;
  static const char* BOLD;
  static const char* NAME;
  static const char* OK;
  static const char* WARN;
  static const char* FAIL;
};

class ServiceCallFailure : public std::runtime_error
{
public:
  explicit ServiceCallFailure(const std::string& message) : std::runtime_error(message)
  {
  }
  const char* what() const throw() override
  {
    return "Service call failed!";
  }
};

class CriticalServiceCallFailure : public ServiceCallFailure
{
public:
  explicit CriticalServiceCallFailure(const std::string& message) : ServiceCallFailure(message)
  {
  }
};

class RLLMoveClientBase
{
public:
  explicit RLLMoveClientBase() = default;
  virtual ~RLLMoveClientBase() = default;

  void setVerbose(bool verbose)
  {
    verbose_ = verbose;
  }

  void setExceptionOnAnyFailure(bool should_throw)
  {
    exception_on_any_failure_ = should_throw;
  }

protected:
  bool verbose_ = true;
  ros::NodeHandle nh_;
  RLLErrorCode last_error_code_ = RLLErrorCode::NOT_SET;

  template <class Request>
  void logServiceCall(const std::string& srv_name, const Request& request);

  template <class SrvMsg>
  bool callServiceWithErrorCode(const std::string& srv_name, ros::ServiceClient srv_client, SrvMsg& srv_data);

  template <class SrvMsg>
  bool callServiceWithoutErrorCode(const std::string& srv_name, ros::ServiceClient srv_client, SrvMsg& srv_data);

  virtual bool handleResponseWithErrorCode(const std::string& srv_name, bool call_success, RLLErrorCode error_code);
  virtual bool handleResponseWithoutErrorCode(const std::string& srv_name, bool call_success);

private:
  bool exception_on_any_failure_ = false;
};

template <class Request>
void RLLMoveClientBase::logServiceCall(const std::string& srv_name, const Request& request)
{
  if (verbose_)
  {
    ROS_INFO("%s%s%s requested:", AnsiCodes::NAME, srv_name.c_str(), AnsiCodes::END);
    ROS_INFO_STREAM(request);  // NOLINT
  }
}

template <class SrvMsg>
bool RLLMoveClientBase::callServiceWithErrorCode(const std::string& srv_name, ros::ServiceClient srv_client,
                                                 SrvMsg& srv_data)
{
  logServiceCall(srv_name, srv_data.request);

  bool call_success = srv_client.call(srv_data);
  return handleResponseWithErrorCode(srv_name, call_success, srv_data.response.error_code);
}

template <class SrvMsg>
bool RLLMoveClientBase::callServiceWithoutErrorCode(const std::string& srv_name, ros::ServiceClient srv_client,
                                                    SrvMsg& srv_data)
{
  logServiceCall(srv_name, srv_data.request);
  bool call_success = srv_client.call(srv_data);
  return handleResponseWithoutErrorCode(srv_name, call_success);
}

class RLLBasicMoveClient : public virtual RLLMoveClientBase
{
public:
  explicit RLLBasicMoveClient();

  bool moveRandom()
  {
    geometry_msgs::Pose pose;
    return moveRandom(&pose);
  }
  bool moveRandom(geometry_msgs::Pose* result_pose);
  bool movePTP(const geometry_msgs::Pose& pose);
  bool moveLin(const geometry_msgs::Pose& pose);

  bool moveJoints(const std::vector<double>& joint_values);
  bool moveJoints(double a1, double a2, double a3, double a4, double a5, double a6, double a7);

protected:
  ros::ServiceClient move_joints_;
  ros::ServiceClient move_ptp_;
  ros::ServiceClient move_lin_;
  ros::ServiceClient move_random_;
};

class RLLGetPoseMoveClient : public virtual RLLMoveClientBase
{
public:
  explicit RLLGetPoseMoveClient()
    : RLLMoveClientBase()
    , get_joint_values_(nh_.serviceClient<rll_msgs::GetJointValues>(RLLMoveIface::GET_JOINT_VALUES_SRV_NAME))
    , get_pose_(nh_.serviceClient<rll_msgs::GetPose>(RLLMoveIface::GET_POSE_SRV_NAME))
  {
  }

  bool getCurrentPose(geometry_msgs::Pose* pose);
  bool getCurrentJointValues(std::vector<double>* joint_values);

protected:
  ros::ServiceClient get_pose_;
  ros::ServiceClient get_joint_values_;
};

// TODO(uieai) test pick place
class RLLPickPlaceClient : public virtual RLLMoveClientBase
{
public:
  explicit RLLPickPlaceClient()
    : RLLMoveClientBase(), pick_place_(nh_.serviceClient<rll_msgs::PickPlace>(RLLMoveIface::PICK_PLACE_SRV_NAME))
  {
  }

  bool pickPlace(const geometry_msgs::Pose& pose_above, const geometry_msgs::Pose& pose_grip, bool gripper_close,
                 const std::string& grasp_object);

protected:
  ros::ServiceClient pick_place_;
};

#endif /* RLL_MOVE_CLIENT_H_ */
