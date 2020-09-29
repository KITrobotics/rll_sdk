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

#ifndef RLL_MOVE_MOVE_IFACE_SERVICES_H
#define RLL_MOVE_MOVE_IFACE_SERVICES_H

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <rll_move/move_iface_planning.h>
#include <rll_move/move_iface_state_machine.h>
#include <rll_move/permissions.h>
#include <rll_msgs/GetJointValues.h>
#include <rll_msgs/GetPose.h>
#include <rll_msgs/MoveJoints.h>
#include <rll_msgs/MoveLin.h>
#include <rll_msgs/MoveLinArmangle.h>
#include <rll_msgs/MovePTP.h>
#include <rll_msgs/MovePTPArmangle.h>
#include <rll_msgs/MoveRandom.h>

#define RLL_SRV_TRUE 1U
#define RLL_SRV_FALSE 0U

// TODO(uieai): extract a Services base class, which does not inherit from Planning?
class RLLMoveIfaceServices : public virtual RLLMoveIfacePlanning
{
public:
  RLLMoveIfaceServices();

  // Available services names
  static const std::string ROBOT_READY_SRV_NAME;
  static const std::string MOVE_PTP_SRV_NAME;
  static const std::string MOVE_PTP_ARMANGLE_SRV_NAME;
  static const std::string MOVE_LIN_SRV_NAME;
  static const std::string MOVE_LIN_ARMANGLE_SRV_NAME;
  static const std::string MOVE_JOINTS_SRV_NAME;
  static const std::string MOVE_RANDOM_SRV_NAME;
  static const std::string GET_POSE_SRV_NAME;
  static const std::string GET_JOINT_VALUES_SRV_NAME;

  virtual RLLErrorCode resetToHome();

  // the public interface exposes only service end points
  // NOLINTNEXTLINE(google-runtime-references)
  bool robotReadySrv(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  // NOLINTNEXTLINE(google-runtime-references)
  bool moveLinSrv(rll_msgs::MoveLin::Request& req, rll_msgs::MoveLin::Response& resp);
  // NOLINTNEXTLINE(google-runtime-references)
  bool moveLinArmangleSrv(rll_msgs::MoveLinArmangle::Request& req, rll_msgs::MoveLinArmangle::Response& resp);
  // NOLINTNEXTLINE(google-runtime-references)
  bool movePTPSrv(rll_msgs::MovePTP::Request& req, rll_msgs::MovePTP::Response& resp);
  // NOLINTNEXTLINE(google-runtime-references)
  bool movePTPArmangleSrv(rll_msgs::MovePTPArmangle::Request& req, rll_msgs::MovePTPArmangle::Response& resp);
  // NOLINTNEXTLINE(google-runtime-references)
  bool moveJointsSrv(rll_msgs::MoveJoints::Request& req, rll_msgs::MoveJoints::Response& resp);

  // NOLINTNEXTLINE(google-runtime-references)
  bool moveRandomSrv(rll_msgs::MoveRandom::Request& req, rll_msgs::MoveRandom::Response& resp);
  // NOLINTNEXTLINE(google-runtime-references)
  bool getCurrentPoseSrv(rll_msgs::GetPose::Request& req, rll_msgs::GetPose::Response& resp);

  // NOLINTNEXTLINE(google-runtime-references)
  bool getCurrentJointValuesSrv(rll_msgs::GetJointValues::Request& req, rll_msgs::GetJointValues::Response& resp);

protected:
  RLLMoveIfaceStateMachine iface_state_;
  Permissions permissions_;
  Permissions::Index move_permission_;
  Permissions::Index only_during_job_run_permission_;

  void setupPermissions();

  void abortDueToCriticalFailure() override;

  RLLErrorCode moveLin(const rll_msgs::MoveLin::Request& req, rll_msgs::MoveLin::Response* resp);
  RLLErrorCode moveLinArmangle(const rll_msgs::MoveLinArmangle::Request& req,
                               rll_msgs::MoveLinArmangle::Response* resp);
  RLLErrorCode movePTP(const rll_msgs::MovePTP::Request& req, rll_msgs::MovePTP::Response* resp);
  RLLErrorCode movePTPArmangle(const rll_msgs::MovePTPArmangle::Request& req,
                               rll_msgs::MovePTPArmangle::Response* resp);
  RLLErrorCode moveJoints(const rll_msgs::MoveJoints::Request& req, rll_msgs::MoveJoints::Response* resp);
  RLLErrorCode moveRandom(const rll_msgs::MoveRandom::Request& req, rll_msgs::MoveRandom::Response* resp);
  RLLErrorCode getCurrentPose(const rll_msgs::GetPose::Request& req, rll_msgs::GetPose::Response* resp);
  RLLErrorCode getCurrentJointValues(const rll_msgs::GetJointValues::Request& req,
                                     rll_msgs::GetJointValues::Response* resp);

  virtual RLLErrorCode beforeServiceCall(const std::string& srv_name);
  virtual RLLErrorCode afterServiceCall(const std::string& srv_name, const RLLErrorCode& previous_error_code);

  template <class Request, class Response, class BaseClass>
  bool controlledMovementExecution(const Request& req, Response* resp, const std::string& srv_name,
                                   RLLErrorCode (BaseClass::*move_func)(const Request&, Response*));

  void handleFailureSeverity(const RLLErrorCode& error_code);
};

template <class Request, class Response, class BaseClass>
bool RLLMoveIfaceServices::controlledMovementExecution(const Request& req, Response* resp, const std::string& srv_name,
                                                       RLLErrorCode (BaseClass::*move_func)(const Request&, Response*))
{
  RLLErrorCode error_code = beforeServiceCall(srv_name);

  // only execute the move_func if the prior check succeeded
  if (error_code.succeeded())
  {
    // we need to cast the 'this' pointer to a (possibly) derived class. Since we use
    // virtual inheritance we cannot use static_cast
    auto iface_ptr = dynamic_cast<BaseClass*>(this);
    if (iface_ptr == nullptr)
    {
      ROS_FATAL("controlledMovementExecution called with an invalid move function!");
      error_code = RLLErrorCode::INTERNAL_ERROR;
    }
    else
    {
      error_code = (iface_ptr->*move_func)(req, resp);
    }
  }

  error_code = afterServiceCall(srv_name, error_code);

  resp->error_code = error_code.value();
  resp->success = error_code.succeeded();

  return true;
}

#endif  // RLL_MOVE_MOVE_IFACE_SERVICES_H

/*
 * Local Variables:
 * c-file-style: "google"
 * End:
 */
