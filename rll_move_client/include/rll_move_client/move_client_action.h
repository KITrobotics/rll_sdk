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

#ifndef MOVE_CLIENT_ACTION_H_
#define MOVE_CLIENT_ACTION_H_

#include <rll_move_client/move_client.h>

#include <actionlib/server/simple_action_server.h>
#include <rll_msgs/DefaultMoveIfaceAction.h>
#include <rll_msgs/DefaultMoveIfaceActionGoal.h>

class RLLActionMoveClient : public virtual RLLMoveClientBase
{
public:
  explicit RLLActionMoveClient(const std::string& name);

protected:
  bool virtual execute() = 0;

private:
  // From ROS-Wiki: "NodeHandle instance must be created before this line Otherwise strange error occurs."
  actionlib::SimpleActionServer<rll_msgs::DefaultMoveIfaceAction> server_;
  void executeCallback(const rll_msgs::DefaultMoveIfaceGoalConstPtr& goal);
};

#endif /* MOVE_CLIENT_ACTION_H_ */
