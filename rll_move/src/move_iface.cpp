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

#include <rll_move/move_iface.h>
#include "moveit/planning_scene_interface/planning_scene_interface.h"

RLLMoveIface::RLLMoveIface()
	: manip_move_group(MANIP_PLANNING_GROUP),
	  gripper_move_group(GRIPPER_PLANNING_GROUP)
{
	manip_move_group.setPlannerId("RRTConnectkConfigDefault");
	manip_move_group.setPlanningTime(2.0);
	manip_move_group.setPoseReferenceFrame("world");
	gripper_move_group.setPlannerId("RRTConnectkConfigDefault");
	gripper_move_group.setPlanningTime(2.0);

	std::string ee_link = "iiwa_gripper_link_ee";
	manip_move_group.setEndEffectorLink(ee_link);

	manip_model = manip_move_group.getRobotModel();
	planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor = planning_scene_monitor::PlanningSceneMonitorPtr(
		new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

	reset_to_home();
	bool success = open_gripper();
	if (!success)
		ROS_ERROR("init: failed to open gripper!");
}

bool RLLMoveIface::pick_place(rll_msgs::PickPlace::Request &req,
			      rll_msgs::PickPlace::Response &resp)
{
	bool success;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	std::vector<geometry_msgs::Pose> waypoints_to;
	std::vector<geometry_msgs::Pose> waypoints_grip;
	std::vector<geometry_msgs::Pose> waypoints_away;
	moveit_msgs::RobotTrajectory trajectory;
	const double eef_step = 0.001;
	const double jump_threshold = 1000.0;

	ROS_INFO("Moving above target");
	manip_move_group.setStartStateToCurrentState();
	waypoints_to.push_back(req.pose_above);
	double achieved = manip_move_group.computeCartesianPath(waypoints_to,
								eef_step, jump_threshold, trajectory);
	if (achieved < 1 && achieved > 0) {
		ROS_ERROR("only achieved to compute %f of the requested path", achieved);
		resp.success = false;
		return true;
	} else if (achieved <= 0) {
		ROS_ERROR("path planning completely failed");
		resp.success = false;
		return true;
	}

	my_plan.trajectory_= trajectory;

	success = (manip_move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (!success) {
		ROS_ERROR("path execution failed");
		resp.success = false;
		return true;
	}

	ROS_INFO("Moving to grip position");
	manip_move_group.setStartStateToCurrentState();
	waypoints_grip.push_back(req.pose_grip);
	achieved = manip_move_group.computeCartesianPath(waypoints_grip,
							 eef_step, jump_threshold, trajectory);
	if (achieved < 1 && achieved > 0) {
		ROS_ERROR("only achieved to compute %f of the requested path", achieved);
		resp.success = false;
		return true;
	} else if (achieved <= 0) {
		ROS_ERROR("path planning completely failed");
		resp.success = false;
		return true;
	}

	my_plan.trajectory_= trajectory;

	success = (manip_move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (!success) {
		ROS_ERROR("path execution failed");
		resp.success = false;
		return true;
	}

	if (req.gripper_close) {
		attach_grasp_object(req.grasp_object);
		success = close_gripper();
	} else {
		success = open_gripper();
	}

	if (!success) {
		resp.success = false;
		return true;
	}

	ROS_INFO("Moving above grip position");
	manip_move_group.setStartStateToCurrentState();
	waypoints_away.push_back(req.pose_above);
	achieved = manip_move_group.computeCartesianPath(waypoints_away, eef_step, jump_threshold, trajectory);
	if (achieved < 1 && achieved > 0) {
		ROS_ERROR("only achieved to compute %f of the requested path", achieved);
		resp.success = false;
		return true;
	} else if (achieved <= 0) {
		ROS_ERROR("path planning completely failed");
		resp.success = false;
		return true;
	}

	my_plan.trajectory_= trajectory;

	success = (manip_move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (!success) {
		ROS_ERROR("path execution failed");
		resp.success = false;
		return true;
	}

	resp.success = true;
	return true;
}

bool RLLMoveIface::move_lin(rll_msgs::MoveLin::Request &req,
			    rll_msgs::MoveLin::Response &resp)
{
	bool success;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	std::vector<geometry_msgs::Pose> waypoints;
	moveit_msgs::RobotTrajectory trajectory;
	const double eef_step = 0.001;
	const double jump_threshold = 1000.0;

	ROS_INFO("Lin motion requested");
	manip_move_group.setStartStateToCurrentState();
	waypoints.push_back(req.pose);
	double achieved = manip_move_group.computeCartesianPath(waypoints,
								eef_step, jump_threshold, trajectory);
	if (achieved < 1 && achieved > 0) {
		ROS_ERROR("only achieved to compute %f of the requested path", achieved);
		resp.success = false;
		return true;
	} else if (achieved <= 0) {
		ROS_ERROR("path planning completely failed");
		resp.success = false;
		return true;
	}

	my_plan.trajectory_= trajectory;

	success = (manip_move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (!success) {
		ROS_ERROR("path execution failed");
		resp.success = false;
		return true;
	}

	resp.success = true;
	return true;
}

bool RLLMoveIface::move_joints(rll_msgs::MoveJoints::Request &req,
			       rll_msgs::MoveJoints::Response &resp)
{
	bool success;
	std::vector<double> joints;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	ROS_INFO("Joint motion requested");

	manip_move_group.getCurrentState()->copyJointGroupPositions(manip_move_group.getCurrentState()->getRobotModel()->getJointModelGroup(manip_move_group.getName()), joints);
	joints[0] = req.joint_1;
	joints[1] = req.joint_2;
	joints[2] = req.joint_3;
	joints[3] = req.joint_4;
	joints[4] = req.joint_5;
	joints[5] = req.joint_6;
	joints[6] = req.joint_7;

	manip_move_group.setStartStateToCurrentState();
	success = manip_move_group.setJointValueTarget(joints);
	if (!success) {
		ROS_ERROR("requested joint values out of range");
		resp.success = false;
		return true;
	}

	success = (manip_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (!success) {
		ROS_ERROR("path planning failed");
		resp.success = false;
		return true;
	}

	success = (manip_move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (!success) {
		ROS_ERROR("path execution failed");
		resp.success = false;
		return true;
	}

	resp.success = true;
	return true;
}

bool RLLMoveIface::run_trajectory(moveit::planning_interface::MoveGroupInterface &move_group, bool info)
{
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success;

	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (info)
		ROS_INFO("Planning result: %s",
			 success ? "SUCCEEDED" : "FAILED");

	if (success) {
		success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		if (info)
			ROS_INFO("Plan execution result: %s",
				 success ? "SUCCEEDED" : "FAILED");
		if (!success)
			return false;
	} else {
		ROS_WARN("Not executing because planning failed");
		return false;
	}

	return true;
}

bool RLLMoveIface::close_gripper()
{
	bool info = true;

	ROS_INFO("Closing the gripper");

	gripper_move_group.setStartStateToCurrentState();
	gripper_move_group.setNamedTarget("gripper_close");
	bool success = run_trajectory(gripper_move_group, info);
	if (!success)
		return false;

	return true;
}

bool RLLMoveIface::open_gripper()
{
	bool info = true;

	ROS_INFO("Opening the gripper");

	gripper_move_group.setStartStateToCurrentState();
	gripper_move_group.setNamedTarget("gripper_open");
	bool success = run_trajectory(gripper_move_group, info);
	if (!success)
		return false;

	return true;
}

bool RLLMoveIface::reset_to_home(bool info)
{
	if (info)
		ROS_INFO("Moving to home");

	manip_move_group.setStartStateToCurrentState();
	manip_move_group.setNamedTarget("home_bow");
	bool success = run_trajectory(manip_move_group, info);
	if (!success)
		return false;

	return true;
}

bool RLLMoveIface::attach_grasp_object(std::string object_id)
{
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit_msgs::CollisionObject remove_object;

	if (object_id.empty())
		return true;

	ROS_INFO("attaching grasp object '%s'", object_id.c_str());

	std::map<std::string,moveit_msgs::CollisionObject> objects = planning_scene_interface.getObjects(std::vector<std::string>{ object_id });

	if (!objects.empty())
		remove_object = objects[object_id];
	else {
		ROS_ERROR("object not found");
		return false;
	}

	if (remove_object.id != object_id) {
		ROS_ERROR("The found grasp object is not the right one");
		return false;
	}

	remove_object.operation = remove_object.REMOVE;

	planning_scene_interface.applyCollisionObject(remove_object);

	moveit_msgs::AttachedCollisionObject attached_object;
	attached_object.link_name = manip_move_group.getEndEffectorLink();
	attached_object.object = remove_object;
	attached_object.object.operation = attached_object.object.ADD;
	// TODO: account for different robot names (if it's not "iiwa")
	attached_object.touch_links = std::vector<std::string>{ "iiwa_gripper_finger_left", "iiwa_gripper_finger_right", "table" };
	planning_scene_interface.applyAttachedCollisionObject(attached_object);
}

RLLMoveIface::~RLLMoveIface() {}

/*
 * Local Variables:
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * End:
 */
