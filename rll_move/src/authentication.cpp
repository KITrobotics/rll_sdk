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

#include <rll_move/authentication.h>
#include <ros/ros.h>

bool Authentication::authenticate(const std::string& authenticate_with)
{
  if (!authentication_required_)
  {
    if (authenticate_with.length() != 0)
    {
      ROS_WARN("No authentication required, but a secret was specified.");
    }
    else
    {
      ROS_INFO("No authentication required");
    }

    return true;
  }

  if (authenticate_with.length() == 0)
  {
    ROS_INFO("No authentication secret specified.");
    return false;
  }

  bool match = authenticate_with == secret_;
  ROS_INFO("Authentication %s!", match ? "succeed" : "failed");

  return match;
}

void Authentication::setSecret(const std::string& auth_token)
{
  secret_ = auth_token;
  authentication_required_ = secret_.length() > 0;

  ROS_INFO("Authentication %s", authentication_required_ ? "enabled" : "disabled");
}
