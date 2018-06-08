/*
 * This file is part of the Robot Learning Lab SDK
 *
 * Copyright (C) 2018 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
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

#ifndef RLL_MOVE_IFACE_H
#define RLL_MOVE_IFACE_H

#include <ros/ros.h>

#include <rll_msgs/PickPlace.h>
#include <rll_msgs/MoveLin.h>
#include <rll_msgs/MoveJoints.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "moveit/planning_scene_interface/planning_scene_interface.h"

class RLLMoveIface
{
public:
	explicit RLLMoveIface();

	const std::string MANIP_PLANNING_GROUP = "manipulator";
	const std::string GRIPPER_PLANNING_GROUP = "gripper";
	moveit::planning_interface::MoveGroupInterface manip_move_group;
	moveit::planning_interface::MoveGroupInterface gripper_move_group;
	moveit::core::RobotModelConstPtr manip_model;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	bool pick_place(rll_msgs::PickPlace::Request &req,
			rll_msgs::PickPlace::Response &resp);
	bool move_lin(rll_msgs::MoveLin::Request &req,
		      rll_msgs::MoveLin::Response &resp);
	bool move_joints(rll_msgs::MoveJoints::Request &req,
			 rll_msgs::MoveJoints::Response &resp);
	bool reset_to_home(bool info = true);
	bool close_gripper();
	bool open_gripper();

	~RLLMoveIface();
  
private:
	bool run_trajectory(moveit::planning_interface::MoveGroupInterface &move_group,
			    bool info = true);
	bool attach_grasp_object(std::string object_id);
	bool detach_grasp_object(std::string object_id);
};

#endif  // RLL_MOVE_IFACE_H

/*
 * Local Variables:
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * End:
 */
