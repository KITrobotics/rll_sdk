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

#include <move_iface.h>

RLLMoveIface::RLLMoveIface(ros::NodeHandle nh)
	: manip_move_group(MANIP_PLANNING_GROUP),
	  gripper_move_group(GRIPPER_PLANNING_GROUP)
{
	manip_move_group.setPlannerId("RRTConnectkConfigDefault");
	manip_move_group.setPlanningTime(2.0);
	manip_move_group.setPoseReferenceFrame("world");
	gripper_move_group.setPlannerId("RRTConnectkConfigDefault");
	gripper_move_group.setPlanningTime(2.0);

	std::string ee_link =  "iiwa_gripper_link_ee";
	manip_move_group.setEndEffectorLink(ee_link);

	reset_to_home();
	open_gripper();
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

	if (req.gripper_close)
		close_gripper();
	else
		open_gripper();

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
	bool success_plan;

	success_plan = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (info)
		ROS_INFO("Planning result: %s",
			 success_plan ? "SUCCEEDED" : "FAILED");

	if (success_plan) {
		success_plan = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		if (info)
			ROS_INFO("Plan execution result: %s",
				 success_plan ? "SUCCEEDED" : "FAILED");
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
	run_trajectory(gripper_move_group, info);

	return true;
}

bool RLLMoveIface::open_gripper()
{
	bool info = true;

	ROS_INFO("Opening the gripper");

	gripper_move_group.setStartStateToCurrentState();
	gripper_move_group.setNamedTarget("gripper_open");
	run_trajectory(gripper_move_group, info);

	return true;
}

void RLLMoveIface::reset_to_home(bool info)
{
	if (info)
		ROS_INFO("Moving to home");

	manip_move_group.setStartStateToCurrentState();
	manip_move_group.setNamedTarget("home_bow");
	run_trajectory(manip_move_group, info);
}

RLLMoveIface::~RLLMoveIface() {}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_iface");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(0);
	spinner.start();

	RLLMoveIface move_iface(nh);

	ros::ServiceServer pick_place = nh.advertiseService("pick_place", &RLLMoveIface::pick_place, &move_iface);
	ros::ServiceServer move_lin = nh.advertiseService("move_lin", &RLLMoveIface::move_lin, &move_iface);
	ros::ServiceServer move_joints = nh.advertiseService("move_joints", &RLLMoveIface::move_joints, &move_iface);

	ROS_INFO("RLL Move Interface started");

	ros::waitForShutdown();

	return 0;
}

/*
 * Local Variables:
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * End:
 */
