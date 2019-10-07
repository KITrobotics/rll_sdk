/*
 * This file is part of the Robot Learning Lab SDK
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

#include <rll_move/move_iface.h>
#include <rll_move/move_iface_state_machine.h>
#include <ros/ros.h>
#include <string>

void RLLMoveIfaceStateMachine::enterErrorState()
{
  std::lock_guard<std::mutex> lock(mutex_);

  setStateToInternalError();
}

void RLLMoveIfaceStateMachine::setStateToInternalError()
{
  if (state_ != RLLMoveIfaceState::INTERNAL_ERROR)
  {
    ROS_ERROR("Entering INTERNAL_ERROR state!");
  }

  // allow changing the state even if we are executing a service call
  state_ = RLLMoveIfaceState::INTERNAL_ERROR;
}

bool RLLMoveIfaceStateMachine::enterState(RLLMoveIfaceState new_state)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (state_ != RLLMoveIfaceState::WAITING)
  {
    ROS_ERROR("Can only enter new state %s from WAITING, currently in %s.", stateToString(new_state),
              stateToString(state_));
  }
  else if (concurrent_service_calls_counter_ > 0)
  {
    ROS_ERROR("State changes are not allowed if a service call is currently in execution!");
  }
  else if (setState(new_state))
  {
    return true;
  }

  setStateToInternalError();
  return false;
}

bool RLLMoveIfaceStateMachine::leaveState()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (concurrent_service_calls_counter_ > 0)
  {
    ROS_ERROR("Cannot end state while a service call is currently in execution!");
  }
  else if (setState(RLLMoveIfaceState::WAITING))
  {
    return true;
  }

  setStateToInternalError();
  return false;
}

bool RLLMoveIfaceStateMachine::setState(RLLMoveIfaceState new_state)
{
  // cannot leave INTERNAL_ERROR state once it is entered
  if (state_ == RLLMoveIfaceState::INTERNAL_ERROR)
  {
    ROS_ERROR("Cannot change state, if in INTERNAL_ERROR state!");
    return false;
  }

  ROS_INFO("Changing state from %s to %s.", stateToString(state_), stateToString(new_state));
  state_ = new_state;

  return true;
}

RLLErrorCode RLLMoveIfaceStateMachine::beginServiceCall(const std::string& srv_name, bool only_allowed_during_job_run)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // increment first, validate later. Since endServiceCall() should also be called
  // which will decrement the counter
  concurrent_service_calls_counter_++;

  if (state_ == RLLMoveIfaceState::INTERNAL_ERROR)
  {
    ROS_INFO("Service calls are not allowed if the state is INTERNAL_ERROR!");
    return RLLErrorCode::INTERNAL_ERROR;
  }

  if (state_ != RLLMoveIfaceState::RUNNING_JOB)
  {
    // some service calls may be allowed outside a job run, these require the state at least to be WAITING
    if (only_allowed_during_job_run || (state_ != RLLMoveIfaceState::WAITING))
    {
      ROS_ERROR("Invalid state: service call %s is not allowed in state %s.", srv_name.c_str(), stateToString(state_));
      return RLLErrorCode::SERVICE_CALL_NOT_ALLOWED;
    }
  }

  if (concurrent_service_calls_counter_ > 1)
  {
    ROS_ERROR("Concurrent service calls are not allowed! Concurrent calls. %d", concurrent_service_calls_counter_);
    return RLLErrorCode::CONCURRENT_SERVICE_CALL;
  }

  return RLLErrorCode::SUCCESS;
}

bool RLLMoveIfaceStateMachine::setCurrentServiceCallResult(RLLErrorCode error_code)
{
  std::lock_guard<std::mutex> service_call_lock(mutex_);

  if (concurrent_service_calls_counter_ > 0)
  {
    override_service_call_result_ = error_code;
    return true;
  }

  return false;
}

RLLErrorCode RLLMoveIfaceStateMachine::endServiceCall(std::string srv_name, bool only_allowed_during_job_run)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (concurrent_service_calls_counter_ <= 0)
  {
    ROS_ERROR("No service call to end. This should not happen!");
    setStateToInternalError();  // somebody is using it wrong
    return RLLErrorCode::INTERNAL_ERROR;
  }

  // always decrement even in case of an error
  concurrent_service_calls_counter_--;

  if (state_ == RLLMoveIfaceState::INTERNAL_ERROR)
  {
    ROS_INFO("Ending service call during INTERNAL_ERROR state.");
    return RLLErrorCode::INTERNAL_ERROR;
  }

  // some service calls may be allowed outside a job run
  if (state_ != RLLMoveIfaceState::RUNNING_JOB)
  {
    // some service calls may be allowed outside a job run
    if (only_allowed_during_job_run || (state_ != RLLMoveIfaceState::WAITING))
    {
      ROS_ERROR("Invalid state: service call %s cannot be ended in state %s!", srv_name.c_str(), stateToString(state_));
      return RLLErrorCode::SERVICE_CALL_NOT_ALLOWED;
    }
  }

  if (override_service_call_result_.value() != RLLErrorCode::NOT_SET)
  {
    // in some cases we want to change the result, e.g. when the service call timed out
    RLLErrorCode tmp = override_service_call_result_;
    override_service_call_result_ = RLLErrorCode::NOT_SET;
    return tmp;
  }

  return RLLErrorCode::SUCCESS;
}
