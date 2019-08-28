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

#ifndef INCLUDE_RLL_MOVE_MOVE_IFACE_BASE_H_
#define INCLUDE_RLL_MOVE_MOVE_IFACE_BASE_H_

#include <rll_move/move_iface.h>
#include <rll_msgs/DefaultMoveIfaceAction.h>

// NOTE: to avoid making RLLMoveIface a template class, RLLMoveIfaceBase is introduced as an intermediary class.
// It should be used as the base class for implementing custom interfaces
template <class Action = rll_msgs::DefaultMoveIfaceAction, class Goal = rll_msgs::DefaultMoveIfaceGoal>
class RLLMoveIfaceBase : public virtual RLLMoveIface
{
public:
  RLLMoveIfaceBase(ros::NodeHandle nh, const std::string& action_name)
    : RLLMoveIface(), nh_(nh), action_client_(action_name, false), action_client_ptr_(&action_client_)
  {
  }

  // since there will be a sim/real version, setup the services/actions in here und make sure to call spin()
  // or use global service objects to keep them alive
  virtual void startServicesAndRunNode(ros::NodeHandle& nh) = 0;

protected:
  ros::NodeHandle nh_;

  void runJob(const rll_msgs::JobEnvGoalConstPtr& goal, rll_msgs::JobEnvResult& result) override;

  void abortDueToCriticalFailure() override
  {
    RLLMoveIface::abortDueToCriticalFailure();
    action_client_ptr_->cancelAllGoals();
  }

  actionlib::SimpleActionClient<Action>* const action_client_ptr_;
  actionlib::SimpleActionClient<Action> action_client_;
};

// the definition for this function needs to be in the header
// since the compiler requires the full defintion for template deduction
template <class Action, class Goal>
void RLLMoveIfaceBase<Action, Goal>::runJob(const rll_msgs::JobEnvGoalConstPtr& /*goal*/,
                                            rll_msgs::JobEnvResult& result)
{
  Goal goal_iface_client;

  ROS_INFO("got job running request");

  if (!action_client_ptr_->waitForServer(ros::Duration(4.0)))
  {
    ROS_ERROR("action server not available");
    result.job.status = rll_msgs::JobStatus::FAILURE;
    return;
  }

  action_client_ptr_->sendGoal(goal_iface_client);
  ROS_INFO("called the interface client");

  // wait a maximum of 8 minutes
  bool success = action_client_ptr_->waitForResult(ros::Duration(480.0));
  if (!allowed_to_move_)
  {
    // This is the default job runner and should only be used for demos or testing
    // Assume an internal error if something fails
    ROS_FATAL("Error during current job execution, assuming internal error");
    result.job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
  }
  else if (!success)
  {
    result.job.status = rll_msgs::JobStatus::FAILURE;
  }
  else
  {
    result.job.status = rll_msgs::JobStatus::SUCCESS;
  }
}

#endif /* INCLUDE_RLL_MOVE_MOVE_IFACE_BASE_H_ */
