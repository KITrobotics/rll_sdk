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

#include <moveit_msgs/MoveGroupAction.h>
#include <actionlib/client/simple_action_client.h>

#include <rll_move/move_iface_base.h>

const std::string RLLMoveIfaceBase::IDLE_JOB_SRV_NAME = "job_idle";
const std::string RLLMoveIfaceBase::RUN_JOB_SRV_NAME = "job_env";

const std::string RLLMoveIfaceBase::JOB_FINISHED_SRV_NAME = "job_finished";

const int RLLMoveIfaceBase::CLIENT_SERVER_PORT = 5005;
const int RLLMoveIfaceBase::CLIENT_SERVER_BUFFER_SIZE = 10;
const char* RLLMoveIfaceBase::CLIENT_SERVER_START_CMD_ = "start";
const char* RLLMoveIfaceBase::CLIENT_SERVER_OK_RESP_ = "ok";
const char* RLLMoveIfaceBase::CLIENT_SERVER_ERROR_RESP_ = "error";

RLLMoveIfaceBase::RLLMoveIfaceBase(const ros::NodeHandle& nh) : nh_(nh)
{
  // set the secret (if any) that is required to invoke actions
  std::string secret;
  ros::param::get(node_name_ + "/authentication_secret", secret);
  authentication_.setSecret(secret);

  bool retrieved = ros::param::get(node_name_ + "/job_execution_timeout", job_execution_timeout_seconds_);
  if (retrieved)
  {
    ROS_INFO("job execution timeout changed to %ds", job_execution_timeout_seconds_);
  }

  client_serv_addr_.sin_family = AF_INET;
  client_serv_addr_.sin_port = htons(CLIENT_SERVER_PORT);
}

void RLLMoveIfaceBase::runJobAction(const rll_msgs::JobEnvGoalConstPtr& goal, JobServer* as)
{
  rll_msgs::JobEnvResult result;

  if (!beforeActionExecution(RLLMoveIfaceState::RUNNING_JOB, goal->authentication_secret, &result))
  {
    // don't attempt to perform an action if we are in the INTERNAL_ERROR state
    as->setSucceeded(result);
    return;
  }

  // run the actual job processing and set the result accordingly
  runJob(goal, result);

  afterActionExecution(&result);
  as->setSucceeded(result);
}

void RLLMoveIfaceBase::idleAction(const rll_msgs::JobEnvGoalConstPtr& goal, JobServer* as)
{
  rll_msgs::JobEnvResult result;

  if (!beforeActionExecution(RLLMoveIfaceState::IDLING, goal->authentication_secret, &result))
  {
    // don't attempt to perform the action we are in the INTERNAL_ERROR state
    as->setSucceeded(result);
    return;
  }

  ROS_INFO("got idle request");
  RLLErrorCode error_code = idle();

  if (error_code.succeeded())
  {
    ROS_INFO("Idle succeeded!");
    result.job.status = rll_msgs::JobStatus::SUCCESS;
  }
  else
  {
    ROS_FATAL("Idle resulted in a %s failure!", error_code.isCriticalFailure() ? "critical" : "recoverable");
    // idle should not fail, so its better to abort further operations
    abortDueToCriticalFailure();
    result.job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
  }

  afterActionExecution(&result);
  as->setSucceeded(result);
}

bool RLLMoveIfaceBase::beforeActionExecution(RLLMoveIfaceState state, const std::string& secret,
                                             rll_msgs::JobEnvResult* result)
{
  // before an action is executed we need to ensure that the caller is authorized to invoke an action
  if (!authentication_.authenticate(secret))
  {
    ROS_FATAL("Authentication is required to execute an action!");
    result->job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
    return false;
  }

  // try to enter the corresponding state
  if (!iface_state_.enterState(state))
  {
    result->job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
    return false;
  }

  return true;
}

bool RLLMoveIfaceBase::afterActionExecution(rll_msgs::JobEnvResult* result)
{
  if (!iface_state_.leaveState())
  {
    result->job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
    return false;
  }

  job_result_.reset();

  return true;
}

// this method should usually be overwritten by projects to add their own routines
void RLLMoveIfaceBase::runJob(const rll_msgs::JobEnvGoalConstPtr& goal, rll_msgs::JobEnvResult& result)
{
  // set the general movement permission for the duration of the job execution
  permissions_.storeCurrentPermissions();
  permissions_.updateCurrentPermissions(move_permission_ | only_during_job_run_permission_ | pick_place_permission_,
                                        true);
  runClient(goal, result);
  permissions_.restorePreviousPermissions();
}

RLLErrorCode RLLMoveIfaceBase::idle()
{
  RLLErrorCode error_code = resetToHome();

  if (error_code.failed())
  {
    ROS_ERROR("resetToHome failed with error_code: %s", error_code.message());
    // escalate the error to ensure it is treated as a critical error
    // it might only be a planning failure but resetToHome may never fail
    return RLLErrorCode::RESET_TO_HOME_FAILED;
  }

  return RLLErrorCode::SUCCESS;
}

bool RLLMoveIfaceBase::runClient(const rll_msgs::JobEnvGoalConstPtr& goal, rll_msgs::JobEnvResult& result)
{
  bool success = initClientSocket(goal->client_ip_addr);
  if (!success)
  {
    result.job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
    return false;
  }

  success = callClient();
  if (!success)
  {
    ROS_WARN("failed to call the interface client");
    result.job.status = rll_msgs::JobStatus::FAILURE;
    return false;
  }

  ROS_INFO("called the interface client");

  ros::Time job_start = ros::Time::now();
  while ((ros::Time::now() - job_start).toSec() < job_execution_timeout_seconds_)
  {
    if (job_result_.isSet())
    {
      break;
    }

    if (iface_state_.isInInternalErrorState())
    {
      break;
    }

    ros::Duration(0.01).sleep();
  }

  if (job_result_.isSet())
  {
    ROS_INFO("interface client completed");
  }
  else if (!iface_state_.isInInternalErrorState())
  {
    ROS_WARN("interface client timed out");
  }

  result.job.status = job_result_.getResult();

  // It is possible that a service call might still be in execution
  // therefore wait for the service call to end before completing the runJob action
  if (iface_state_.setCurrentServiceCallResult(RLLErrorCode::JOB_EXECUTION_TIMED_OUT))
  {
    while (iface_state_.isServiceCallInExecution())
    {
      ros::Duration(.01).sleep();
    }
  }

  if (iface_state_.isInInternalErrorState())
  {
    ROS_FATAL("Internal error during current job execution!");
    result.job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
    return false;
  }

  // sanity check: if a job fails with an internal error than further operations should have been aborted
  if (result.job.status == rll_msgs::JobStatus::INTERNAL_ERROR && !iface_state_.isInInternalErrorState())
  {
    ROS_FATAL("Job resulted in an INTERNAL_ERROR, but the state is not set accordingly. This should NOT happen!");
    abortDueToCriticalFailure();
    return false;
  }

  return true;
}

bool RLLMoveIfaceBase::jobFinishedSrv(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
  RLLErrorCode error_code = RLLMoveIfaceServices::beforeNonMovementServiceCall(RLLMoveIfaceBase::JOB_FINISHED_SRV_NAME);

  if (error_code.succeeded())
  {
    job_result_.setResult(req.data);
  }

  error_code = afterNonMovementServiceCall(RLLMoveIfaceBase::JOB_FINISHED_SRV_NAME, error_code);
  resp.success = error_code.succeeded();
  return true;
}

bool RLLMoveIfaceBase::initClientSocket(const std::string& client_ip_addr)
{
  if (inet_pton(AF_INET, client_ip_addr.c_str(), &client_serv_addr_.sin_addr) <= 0)
  {
    ROS_ERROR("Invalid client address");
    return false;
  }

  client_socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (client_socket_ < 0)
  {
    ROS_ERROR("failed to create client socket");
    return false;
  }

  // set a timeout of 2s for reading and writing
  struct timeval timeout;
  timeout.tv_sec = 2;
  timeout.tv_usec = 0;

  if (setsockopt(client_socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<char*>(&timeout), sizeof(timeout)) < 0)
  {
    ROS_ERROR("setsockopt for receive timeout failed");
    return false;
  }

  if (setsockopt(client_socket_, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<char*>(&timeout), sizeof(timeout)) < 0)
  {
    ROS_ERROR("setsockopt for send timeout failed");
    return false;
  }

  return true;
}

bool RLLMoveIfaceBase::callClient()
{
  unsigned int max_retries = 11;
  unsigned int retry_counter = 0;
  char recv_msg[CLIENT_SERVER_BUFFER_SIZE];
  memset(recv_msg, 0, CLIENT_SERVER_BUFFER_SIZE * sizeof(char));
  int send_msg_size = strlen(CLIENT_SERVER_START_CMD_);

  while (retry_counter < max_retries)
  {
    int result =
        connect(client_socket_, reinterpret_cast<struct sockaddr*>(&client_serv_addr_), sizeof(client_serv_addr_));
    if (result < 0 && retry_counter < max_retries)
    {
      ROS_INFO("failed to connect to client, retrying...");
      retry_counter++;
      ros::Duration(3.0).sleep();
    }
    else
    {
      break;
    }
  }
  if (retry_counter == max_retries)
  {
    ROS_WARN("max retries exceeded, failed to connect to client");
    return false;
  }

  int client_sent = send(client_socket_, CLIENT_SERVER_START_CMD_, send_msg_size, 0);
  if (client_sent != send_msg_size)
  {
    ROS_WARN("error sending start message to client");
    close(client_socket_);
    return false;
  }

  recv(client_socket_, recv_msg, CLIENT_SERVER_BUFFER_SIZE, 0);
  close(client_socket_);
  if (strcmp(recv_msg, CLIENT_SERVER_OK_RESP_) == 0)
  {
    job_result_.jobStarted();
    return true;
  }
  if (strcmp(recv_msg, CLIENT_SERVER_ERROR_RESP_) == 0)
  {
    ROS_WARN("client responded with an error");
    return false;
  }

  ROS_WARN("error reading response from client");
  return false;
}

bool waitForMoveGroupAction()
{
  // to ensure that the moveit setup is completely loaded wait for move_group action to become available
  ROS_INFO("Waiting for the 'move_group' action to become available");

  actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> ac("move_group", true);
  if (!ac.waitForServer(ros::Duration(10)))
  {
    ROS_FATAL("'move_group' action is not available! Did you launch all required nodes?");
    return false;
  }
  ROS_INFO("done waiting");

  ros::Duration(1).sleep();
  return true;
}
