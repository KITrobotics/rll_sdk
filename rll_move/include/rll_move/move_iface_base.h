/*
 * This file is part of the Robot Learning Lab SDK
 *
 * Copyright (C) 2018 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
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

#ifndef RLL_MOVE_IFACE_BASE_H_
#define RLL_MOVE_IFACE_BASE_H_

#include <sys/socket.h>
#include <arpa/inet.h>

#include <actionlib/server/simple_action_server.h>

#include <rll_msgs/JobEnvAction.h>
#include <rll_move/move_iface_services.h>
#include <rll_move/authentication.h>

class RLLJobResult
{
public:
  void setResult(bool result)
  {
    success_ = result;
    result_reported_ = true;
    time_job_finished_ = ros::Time::now();
  }

  void jobStarted()
  {
    time_job_started_ = ros::Time::now();
  }

  void reset()
  {
    success_ = false;
    result_reported_ = false;
  }

  bool isSet()
  {
    return result_reported_;
  }

  // returns as RLL job status code
  uint8_t getResult()
  {
    if (result_reported_ && success_)
    {
      return rll_msgs::JobStatus::SUCCESS;
    }

    return rll_msgs::JobStatus::FAILURE;
  }

  ros::Duration getJobDuration()
  {
    if (result_reported_)
    {
      return time_job_finished_ - time_job_started_;
    }

    return ros::DURATION_MAX;
  }

protected:
  bool success_ = false;
  bool result_reported_ = false;
  ros::Time time_job_started_;
  ros::Time time_job_finished_;
};

// TODO(wolfgang): rename this into a project base class?
// RLLMoveIfaceBase should be used as the base class for custom interfaces.
class RLLMoveIfaceBase : public virtual RLLMoveIfaceServices
{
public:
  explicit RLLMoveIfaceBase(const ros::NodeHandle& nh);

  static const std::string IDLE_JOB_SRV_NAME;
  static const std::string RUN_JOB_SRV_NAME;
  static const std::string JOB_FINISHED_SRV_NAME;

  static const int CLIENT_SERVER_PORT;
  static const int CLIENT_SERVER_BUFFER_SIZE;
  static const char* CLIENT_SERVER_START_CMD_;
  static const char* CLIENT_SERVER_OK_RESP_;
  static const char* CLIENT_SERVER_ERROR_RESP_;

  // since there will be a sim/real version, setup the services in here und make sure to call spin()
  // or use global service objects to keep them alive
  virtual void startServicesAndRunNode(ros::NodeHandle& nh) = 0;

  using JobServer = actionlib::SimpleActionServer<rll_msgs::JobEnvAction>;
  void runJobAction(const rll_msgs::JobEnvGoalConstPtr& goal, JobServer* as);
  void idleAction(const rll_msgs::JobEnvGoalConstPtr& goal, JobServer* as);
  bool jobFinishedSrv(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);

protected:
  ros::NodeHandle nh_;
  int client_socket_;
  struct sockaddr_in client_serv_addr_;

  Authentication authentication_;
  RLLJobResult job_result_;

  virtual void runJob(const rll_msgs::JobEnvGoalConstPtr& goal, rll_msgs::JobEnvResult& result);
  virtual bool runClient(const rll_msgs::JobEnvGoalConstPtr& goal, rll_msgs::JobEnvResult& result);
  virtual RLLErrorCode idle();
  bool initClientSocket(const std::string& client_ip_addr);
  bool callClient();

  bool beforeActionExecution(RLLMoveIfaceState state, const std::string& secret, rll_msgs::JobEnvResult* result);
  bool afterActionExecution(rll_msgs::JobEnvResult* result);

  void abortDueToCriticalFailure() override
  {
    RLLMoveIfaceServices::abortDueToCriticalFailure();
  }

  int job_execution_timeout_seconds_ = 600;  // default is ten minutes
};

template <class BaseIface, class EnvironmentIface>
class RLLCombinedMoveIface : public BaseIface, public EnvironmentIface
{
public:
  explicit RLLCombinedMoveIface(ros::NodeHandle nh) : BaseIface(nh), EnvironmentIface()
  {
  }
};

/**
 * \brief Waits for the `move_group` action to become available.
 */
bool waitForMoveGroupAction();

#endif /* RLL_MOVE_IFACE_BASE_H_ */
