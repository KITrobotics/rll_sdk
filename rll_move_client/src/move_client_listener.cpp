/*
 * This file is part of the Robot Learning Lab Move Client
 *
 * Copyright (C) 2019 Mark Weinreuter <uieai@student.kit.edu>
 * Copyright (C) 2019 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
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

#include <sys/socket.h>
#include <arpa/inet.h>

#include <rll_move/move_iface_base.h>
#include <rll_move_client/move_client_listener.h>

RLLMoveClientListener::RLLMoveClientListener()
{
  job_finished_ = nh_.serviceClient<std_srvs::SetBool>(RLLMoveIfaceBase::JOB_FINISHED_SRV_NAME);

  socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_ < 0)
  {
    ROS_ERROR("failed to create socket listener");
    return;
  }

  // set a timeout of 2s for reading and writing
  struct timeval timeout;
  timeout.tv_sec = 2;
  timeout.tv_usec = 0;

  if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<char*>(&timeout), sizeof(timeout)) < 0)
  {
    ROS_ERROR("setsockopt for receive timeout failed");
    return;
  }

  if (setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<char*>(&timeout), sizeof(timeout)) < 0)
  {
    ROS_ERROR("setsockopt for send timeout failed");
    return;
  }

  struct sockaddr_in address;
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(RLLMoveIfaceBase::CLIENT_SERVER_PORT);
  if (bind(socket_, reinterpret_cast<struct sockaddr*>(&address), sizeof(address)) < 0)
  {
    ROS_ERROR("failed to bind socket to port");
    return;
  }

  // only allow one connection
  if (listen(socket_, 1) < 0)
  {
    ROS_ERROR("failed to init socket listener");
  }

  // TODO(uieai): If starting the socket fails, e.g. because another client is already running
  // only the error message gets printed which isn't really helpful. A better error message
  // should be printed and the client shutdown? The python version shuts down with address already in use

  ROS_INFO("Socket listener started");
}

void RLLMoveClientListener::executeCallback()
{
  bool success = false;
  is_job_running_ = true;
  ROS_INFO("Code execution triggered");

  try
  {
    success = execute();
  }
  catch (ServiceCallFailure& e)
  {
    ROS_ERROR("The client routine was interrupted by an uncaught service exception: %s", e.what());
  }
  catch (...)
  {
    // catch all exceptions to make sure the client exits cleanly
    ROS_ERROR("The client routine was interrupted by an uncaught exception.");
  }

  if (success)
  {
    ROS_INFO("Callback completed %ssuccessfully%s", AnsiCodes::OK_, AnsiCodes::END_);
  }
  else
  {
    ROS_INFO("Callback completed %sunsuccessfully%s", AnsiCodes::WARN_, AnsiCodes::END_);
  }

  notifyJobFinished(success);
  ROS_INFO("Code execution completed");
}

void RLLMoveClientListener::notifyJobFinished(bool success)
{
  if (!is_job_running_)
  {
    return;
  }
  is_job_running_ = false;

  std_srvs::SetBool job_finished_msg;
  job_finished_msg.request.data = success;
  bool call_success = job_finished_.call(job_finished_msg);
  if (!call_success || !job_finished_msg.response.success)
  {
    ROS_ERROR("failed to report execution result to interface");
  }
}

void RLLMoveClientListener::spin()
{
  int tmp_socket;
  struct sockaddr_in conn_addr;
  socklen_t conn_len;
  char recv_msg[RLLMoveIfaceBase::CLIENT_SERVER_BUFFER_SIZE];
  memset(recv_msg, 0, RLLMoveIfaceBase::CLIENT_SERVER_BUFFER_SIZE * sizeof(char));

  while (ros::ok())
  {
    tmp_socket = accept(socket_, reinterpret_cast<struct sockaddr*>(&conn_addr), &conn_len);
    if (tmp_socket < 0)
    {
      continue;
    }

    ROS_INFO("got a connection from addr %s and port %d", inet_ntoa(conn_addr.sin_addr), ntohs(conn_addr.sin_port));

    recv(tmp_socket, recv_msg, RLLMoveIfaceBase::CLIENT_SERVER_BUFFER_SIZE, 0);
    if (strcmp(recv_msg, RLLMoveIfaceBase::CLIENT_SERVER_START_CMD_) == 0)
    {
      ROS_INFO("received start signal");
      send(tmp_socket, RLLMoveIfaceBase::CLIENT_SERVER_OK_RESP_, strlen(RLLMoveIfaceBase::CLIENT_SERVER_OK_RESP_), 0);
      executeCallback();
    }
    else
    {
      ROS_ERROR("error receiving start signal");
      send(tmp_socket, RLLMoveIfaceBase::CLIENT_SERVER_ERROR_RESP_, strlen(RLLMoveIfaceBase::CLIENT_SERVER_ERROR_RESP_),
           0);
    }

    close(tmp_socket);
  }
}
