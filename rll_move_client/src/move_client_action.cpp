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

#include <rll_move_client/move_client_action.h>

RLLActionMoveClient::RLLActionMoveClient(const std::string& name)
  : RLLMoveClientBase(), server_(nh_, name, boost::bind(&RLLActionMoveClient::executeCallback, this, _1), false)
{
  ROS_INFO("Action server started");
  server_.start();
}

void RLLActionMoveClient::executeCallback(const rll_msgs::DefaultMoveIfaceGoalConstPtr& /*goal*/)
{
  ROS_INFO("Action triggered");

  try
  {
    bool result = execute();

    if (result)
    {
      ROS_INFO("Callback completed %ssuccessfully%s", AnsiCodes::OK, AnsiCodes::END);
    }
    else
    {
      ROS_INFO("Callback completed %sunsuccessfully%s", AnsiCodes::WARN, AnsiCodes::END);
    }

    server_.setSucceeded();
  }
  catch (ServiceCallFailure& e)
  {
    ROS_ERROR("The action routine was interrupted by an uncaught exception: %s", e.what());
    server_.setAborted();
  }

  ROS_INFO("Action completed");
}
