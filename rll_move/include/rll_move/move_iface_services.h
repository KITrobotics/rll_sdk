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

#ifndef RLL_MOVE_IFACE_SERVICES_H
#define RLL_MOVE_IFACE_SERVICES_H

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <rll_msgs/PickPlace.h>
#include <rll_msgs/MoveLin.h>
#include <rll_msgs/MoveLinElb.h>
#include <rll_msgs/MovePTP.h>
#include <rll_msgs/MovePTPelb.h>
#include <rll_msgs/MoveJoints.h>
#include <rll_msgs/MoveRandom.h>
#include <rll_msgs/GetPose.h>
#include <rll_msgs/GetJointValues.h>
#include <rll_move/move_iface_planning.h>
#include <rll_move/move_iface_state_machine.h>
#include <rll_move/permissions.h>

class RLLMoveIfaceServices : public virtual RLLMoveIfacePlanning
{
public:
  RLLMoveIfaceServices();

  // Available services names
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

  // the public interface exposes only service end points
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
  RLLMoveIfaceStateMachine iface_state_;
  Permissions permissions_;
  Permissions::Index move_permission_;
  Permissions::Index only_during_job_run_permission_;
  Permissions::Index pick_place_permission_;

  void setupPermissions();

  void abortDueToCriticalFailure() override;

  RLLErrorCode pickPlace(rll_msgs::PickPlace::Request& req, rll_msgs::PickPlace::Response& resp);
  RLLErrorCode moveLin(rll_msgs::MoveLin::Request& req, rll_msgs::MoveLin::Response& resp);
  RLLErrorCode moveLinElb(rll_msgs::MoveLinElb::Request& req, rll_msgs::MoveLinElb::Response& resp);
  RLLErrorCode movePTP(rll_msgs::MovePTP::Request& req, rll_msgs::MovePTP::Response& resp);
  RLLErrorCode movePTPelb(rll_msgs::MovePTPelb::Request& req, rll_msgs::MovePTPelb::Response& resp);
  RLLErrorCode moveJoints(rll_msgs::MoveJoints::Request& req, rll_msgs::MoveJoints::Response& resp);
  RLLErrorCode moveRandom(rll_msgs::MoveRandom::Request& req, rll_msgs::MoveRandom::Response& resp);

  virtual RLLErrorCode beforeNonMovementServiceCall(const std::string& srv_name);
  virtual RLLErrorCode afterNonMovementServiceCall(const std::string& srv_name, RLLErrorCode previous_error_code);

  virtual RLLErrorCode beforeMovementServiceCall(const std::string& srv_name);
  virtual RLLErrorCode afterMovementServiceCall(const std::string& srv_name, const RLLErrorCode& previous_error_code);

  template <class Request, class Response, class BaseClass>
  bool controlledMovementExecution(Request& req, Response& resp, const std::string& srv_name,
                                   RLLErrorCode (BaseClass::*move_func)(Request&, Response&));

  void handleFailureSeverity(const RLLErrorCode& error_code);
};

template <class Request, class Response, class BaseClass>
bool RLLMoveIfaceServices::controlledMovementExecution(Request& req, Response& resp, const std::string& srv_name,
                                                       RLLErrorCode (BaseClass::*move_func)(Request&, Response&))
{
  RLLErrorCode error_code = beforeMovementServiceCall(srv_name);

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

  error_code = afterMovementServiceCall(srv_name, error_code);

  resp.error_code = error_code.value();
  resp.success = error_code.succeeded();

  return true;
}

#endif  // RLL_MOVE_IFACE_SERVICES_H

/*
 * Local Variables:
 * c-file-style: "google"
 * End:
 */
