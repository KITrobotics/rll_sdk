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
#include <rll_move_client/move_client_listener.h>

class RLLDefaultMoveClient : public RLLMoveClientListener, public RLLBasicMoveClient, public RLLGetPoseMoveClient
{
public:
  explicit RLLDefaultMoveClient() = default;
};

class RLLDefaultPickPlaceMoveClient : public RLLDefaultMoveClient, public RLLPickPlaceClient
{
public:
  explicit RLLDefaultPickPlaceMoveClient() = default;
};

// if you do not want to derive from RLLMoveClientListener, you can use this template class
// and specify a callback function which will be passed a MoveClient instance
template <class Client>
class RLLCallbackMoveClient : public Client
{
public:
  using ClientCallback = bool (*)(Client* const);
  explicit RLLCallbackMoveClient(ClientCallback exec_func) : Client(), exec_func_(exec_func)
  {
  }

protected:
  ClientCallback exec_func_;
  bool execute() override
  {
    return exec_func_(this);
  }
};

#endif /* MOVE_CLIENT_DEFAULT_H_ */
