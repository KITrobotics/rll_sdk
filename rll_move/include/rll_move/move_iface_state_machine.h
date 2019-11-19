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

#ifndef INCLUDE_RLL_MOVE_MOVE_IFACE_STATE_MACHINE_H_
#define INCLUDE_RLL_MOVE_MOVE_IFACE_STATE_MACHINE_H_

#include <mutex>
#include <rll_move/move_iface_error.h>
#include <rll_move/permissions.h>

enum class RLLMoveIfaceState
{
  WAITING = 1,
  IDLING,
  RUNNING_JOB,
  INTERNAL_ERROR
};

/**
 * Thread safe state management class to keep track of what state the interface is in
 * to determine if an action invocation or service call is currently allowed.
 *
 * The states correspond to what actions are called, i.e. `job_idle` oder `job_env`.
 * Additionally the WAITING and INTERNAL_ERROR states are used to represent that no
 * action is currently being processed or an invocation resulted in an error state.
 *
 * Service calls are only allowed if the internal state is RUNNING_JOB. To keep track of what actions
 * have been triggered and what services invoked call the corresponding begin and end methods.
 *
 * State changes to RUNNING_JOB or IDLING are only allowed from the WAITING state.
 * A transition to the INTERNAL_ERROR state is always possible, but cannot be undone.

 * If a state change is attempted that is not allowed, the INTERNAL_ERROR state is entered.
 * If a service call fails its return value indicates the reason but INTERNAL_ERROR
 * is not automatically entered you will need to do this manually.
 *
 */
class RLLMoveIfaceStateMachine
{
public:
  RLLMoveIfaceStateMachine() = default;

  virtual ~RLLMoveIfaceStateMachine() = default;

  /**
   * \brief Enter the desired state if possible, INTERNAL_ERROR otherwise.
   */
  virtual bool enterState(RLLMoveIfaceState new_state);

  /**
   * \brief End the current state and go into WAITING if possible.
   */
  virtual bool leaveState();

  /**
   * \brief Enters the INTERNAL_ERROR state.
   */
  void enterErrorState();

  /**
   * \brief Notify the state machine that an service call is now in execution.
   */
  virtual RLLErrorCode beginServiceCall(const std::string& srv_name, bool only_allowed_during_job_run);

  /**
   * \brief Notify the state machine that an service call has ended.
   */
  virtual RLLErrorCode endServiceCall(const std::string& srv_name, bool only_allowed_during_job_run);

  /**
   * \brief Indicates if a service call is currently in execution.
   */
  bool isServiceCallInExecution()
  {
    std::lock_guard<std::mutex> service_call_lock(mutex_);
    return concurrent_service_calls_counter_ > 0;
  }

  /**
   * \brief Override the result of a service call if one is currently in execution.
   */
  bool setCurrentServiceCallResult(RLLErrorCode error_code);

  bool isInInternalErrorState()
  {
    std::lock_guard<std::mutex> service_call_lock(mutex_);
    return state_ == RLLMoveIfaceState::INTERNAL_ERROR;
  }

  const char* stateToString(RLLMoveIfaceState state)
  {
    switch (state)
    {
      case RLLMoveIfaceState::WAITING:
        return "WAITING";
      case RLLMoveIfaceState::IDLING:
        return "IDLING";
      case RLLMoveIfaceState::RUNNING_JOB:
        return "RUNNING_JOB";
      case RLLMoveIfaceState::INTERNAL_ERROR:
        return "INTERNAL_ERROR";
      default:
        return "UNKONWN_STATE";
    }
  }

protected:
  // count the number of currently running service requests
  // only one is service call is allowed but this way we can
  // track if there are the same amount of begin()/end() calls
  unsigned int concurrent_service_calls_counter_ = 0;
  RLLMoveIfaceState state_{ RLLMoveIfaceState::WAITING };
  std::mutex mutex_;

private:
  void setStateToInternalError();
  bool setState(RLLMoveIfaceState new_state);
  RLLErrorCode override_service_call_result_;
};

#endif /* INCLUDE_RLL_MOVE_MOVE_IFACE_STATE_MACHINE_H_ */
