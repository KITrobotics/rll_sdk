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

#ifndef RLL_MOVE_CLIENT_MOVE_CLIENT_LISTENER_H
#define RLL_MOVE_CLIENT_MOVE_CLIENT_LISTENER_H

#include <rll_move_client/move_client.h>

class RLLMoveClientListener : public virtual RLLMoveClientBase
{
public:
  explicit RLLMoveClientListener();

  void spin();
  void notifyJobFinished(bool success);

protected:
  bool virtual execute() = 0;

private:
  bool is_job_running_ = false;
  int socket_;
  ros::ServiceClient job_finished_;

  void executeCallback();
};

#endif  // RLL_MOVE_CLIENT_MOVE_CLIENT_LISTENER_H
