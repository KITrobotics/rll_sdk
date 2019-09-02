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

#ifndef RLL_MOVE_IFACE_DEFAULT_H
#define RLL_MOVE_IFACE_DEFAULT_H

#include <rll_move/move_iface_base.h>

class RLLDefaultMoveIfaceBase : public RLLMoveIfaceBase<>
{
public:
  RLLDefaultMoveIfaceBase(ros::NodeHandle nh, const std::string& action_name = "move_client")
    : RLLMoveIfaceBase(nh, action_name)
  {
  }
  void startServicesAndRunNode(ros::NodeHandle& nh) override;
};

#endif  // RLL_MOVE_IFACE_DEFAULT_H

/*
 * Local Variables:
 * c-file-style: "google"
 * End:
 */