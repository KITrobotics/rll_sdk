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

#ifndef MOVE_CLIENT_DEFAULT_H_
#define MOVE_CLIENT_DEFAULT_H_

#include <rll_move_client/move_client.h>
#include <rll_move_client/move_client_action.h>

class RLLDefaultMoveClient : public RLLActionMoveClient, public RLLBasicMoveClient, public RLLGetPoseMoveClient
{
public:
  explicit RLLDefaultMoveClient(const std::string& name)
    : RLLActionMoveClient(name), RLLBasicMoveClient(), RLLGetPoseMoveClient()
  {
  }
};

class RLLDefaultPickPlaceMoveClient : public RLLDefaultMoveClient, public RLLPickPlaceClient
{
public:
  explicit RLLDefaultPickPlaceMoveClient(const std::string& name) : RLLDefaultMoveClient(name), RLLPickPlaceClient()
  {
  }
};

// if you do not want to derive from RLLMoveClientAction, you can use this template class
// and specify a callback function which will be passed a MoveClient instance
template <class Client>
class RLLCallbackMoveClient : public Client
{
public:
  using ClientCallback = void (*)(Client* const);
  RLLCallbackMoveClient(ClientCallback exec_func, const std::string& name) : Client(name), exec_func(exec_func)
  {
  }

protected:
  ClientCallback exec_func;
  bool execute() override
  {
    // ignore the result, to keep the callback as simple as possible
    exec_func(this);
    return true;
  }
};

#endif /* MOVE_CLIENT_DEFAULT_H_ */
