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

#ifndef RLL_MOVE_AUTHENTICATION_H
#define RLL_MOVE_AUTHENTICATION_H

#include <string>

/**
 * Simple authentication by using a shared secret.
 * If the secret isn't set or empty authentication is disabled.
 *
 */
class Authentication
{
public:
  explicit Authentication() = default;
  ~Authentication() = default;

  bool authenticate(const std::string& authenticate_with);
  void setSecret(const std::string& auth_token);

private:
  std::string secret_;
  bool authentication_required_ = false;
};

#endif  // RLL_MOVE_AUTHENTICATION_H
