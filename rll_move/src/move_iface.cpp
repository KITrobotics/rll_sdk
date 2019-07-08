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

RLLMoveIface::RLLMoveIface()
	: manip_move_group(MANIP_PLANNING_GROUP),
	  gripper_move_group(GRIPPER_PLANNING_GROUP)
{
	ns = ros::this_node::getNamespace();
	// remove the slashes at the beginning
#if ROS_VERSION_MINIMUM(1, 14, 3) // Melodic
	ns.erase(0, 1);
#else // Kinetic and older
	ns.erase(0, 2);
#endif
	ROS_INFO("starting in ns %s", ns.c_str());

	std::string node_name = ros::this_node::getName();
	ros::param::get(node_name + "/no_gripper", no_gripper_attached);
	if (no_gripper_attached)
		ROS_INFO("configured to not use a gripper");

	manip_move_group.setPlannerId("RRTConnectkConfigDefault");
	manip_move_group.setPlanningTime(2.0);
	manip_move_group.setPoseReferenceFrame("world");
	gripper_move_group.setPlannerId("RRTConnectkConfigDefault");
	gripper_move_group.setPlanningTime(2.0);

	manip_model = manip_move_group.getRobotModel();

	std::string ee_link = ns + "_gripper_link_ee";
	manip_move_group.setEndEffectorLink(ee_link);

	allowed_to_move = false;
}

void RLLMoveIface::run_job(const rll_msgs::JobEnvGoalConstPtr &goal,
			   JobServer *as)
{
	actionlib::SimpleActionClient<rll_msgs::DefaultMoveIfaceAction> action_client("move_client", true);
	rll_msgs::JobEnvResult result;
	rll_msgs::DefaultMoveIfaceGoal goal_iface_client;

	ROS_INFO("got job running request");

	ros::Duration(0.5).sleep();
	if (!action_client.waitForServer(ros::Duration(4.0))) {
		ROS_ERROR("action service not available");
		result.job.status = rll_msgs::JobStatus::FAILURE;
		as->setSucceeded(result);
		return;
	}

	action_client_ptr = &action_client;

	allowed_to_move = true;
	action_client_ptr->sendGoal(goal_iface_client);
	ROS_INFO("called the interface client");
	// wait a maximum of 8 minutes
	bool success = action_client_ptr->waitForResult(ros::Duration(480.0));
	if (!allowed_to_move) {
		// This is the default job runner and should only be used for demos or testing
		// Assume an internal error if something fails
		ROS_FATAL("Error during current job execution, assuming internal error");
		result.job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
	} else if (!success) {
		result.job.status = rll_msgs::JobStatus::FAILURE;
	} else {
		result.job.status = rll_msgs::JobStatus::SUCCESS;
	}

	allowed_to_move = false;
	as->setSucceeded(result);
}

void RLLMoveIface::idle(const rll_msgs::JobEnvGoalConstPtr &goal,
			JobServer *as)
{
	rll_msgs::JobEnvResult result;

	ROS_INFO("got idle request");

	bool success = reset_to_home();
	if (!success) {
		ROS_ERROR("failed to idle");
		result.job.status = rll_msgs::JobStatus::INTERNAL_ERROR;
		as->setSucceeded(result);
		return;
	}

	result.job.status = rll_msgs::JobStatus::SUCCESS;
	as->setSucceeded(result);
}

bool RLLMoveIface::robot_ready_srv(std_srvs::Trigger::Request &req,
				   std_srvs::Trigger::Response &resp)
{
	if (!reset_to_home())
		resp.success = false;
	else
		resp.success = true;

	return true;
}

bool RLLMoveIface::manip_current_state_available()
{
	// Sometimes, the current state cannot be retrieved and a NULL pointer exception is thrown
	// somewhere. Check here if the current state can be retrieved. Other methods can use this
	// function to abort further MoveIt commands and to avoid sigterms.

	robot_state::RobotStatePtr current_state = manip_move_group.getCurrentState();
	if (current_state == NULL)
		return false;

	return true;
}

bool RLLMoveIface::pick_place_srv(rll_msgs::PickPlace::Request &req,
				  rll_msgs::PickPlace::Response &resp)
{
	if (allowed_to_move) {
		pick_place(req, resp);
	} else {
		ROS_WARN("Not allowed to send pick/place commands");
		resp.success = false;
		return true;
	}

	if (!resp.success) {
		ROS_FATAL("pick_place service call failed");
		allowed_to_move = false;
		action_client_ptr->cancelAllGoals();
	}

	return true;
}

bool RLLMoveIface::pick_place(rll_msgs::PickPlace::Request &req,
			      rll_msgs::PickPlace::Response &resp)
{
	bool success;

	ROS_INFO("Moving above target");
	success = run_lin_trajectory(req.pose_above);
	if (!success) {
		ROS_WARN("Moving above target failed");
		resp.success = false;
		return true;
	}

	ROS_INFO("Moving to grip position");
	success = run_lin_trajectory(req.pose_grip);
	if (!success) {
		ROS_WARN("Moving to grip position failed");
		resp.success = false;
		return true;
	}

	if (req.gripper_close) {
		attach_grasp_object(req.grasp_object);
		success = close_gripper();
	} else {
		success = open_gripper();
		detach_grasp_object(req.grasp_object);
	}

	if (!success) {
		ROS_WARN("Opening or closing the gripper failed");
		resp.success = false;
		return true;
	}

	ROS_INFO("Moving back above grip position");
	success = run_lin_trajectory(req.pose_above);
	if (!success) {
		ROS_WARN("Moving back above target failed");
		resp.success = false;
		return true;
	}

	resp.success = true;
	return true;
}

bool RLLMoveIface::move_lin_srv(rll_msgs::MoveLin::Request &req,
				rll_msgs::MoveLin::Response &resp)
{
	if (allowed_to_move) {
		move_lin(req, resp);
	} else {
		ROS_WARN("Not allowed to send move lin commands");
		resp.success = false;
		return true;
	}

	if (!resp.success) {
		ROS_FATAL("move_lin service call failed");
		allowed_to_move = false;
		action_client_ptr->cancelAllGoals();
	}

	return true;
}

bool RLLMoveIface::move_lin(rll_msgs::MoveLin::Request &req,
			    rll_msgs::MoveLin::Response &resp)
{
	ROS_INFO("Lin motion requested");

	bool success = run_lin_trajectory(req.pose, req.cartesian_time_parametrization);
	if (!success)
		resp.success = false;
	else
		resp.success = true;

	return true;
}

bool RLLMoveIface::move_ptp_srv(rll_msgs::MovePTP::Request &req,
				rll_msgs::MovePTP::Response &resp)
{
	if (allowed_to_move) {
		move_ptp(req, resp);
	} else {
		ROS_WARN("Not allowed to send move ptp commands");
		resp.success = false;
		return true;
	}

	if (!resp.success) {
		ROS_FATAL("move_ptp service call failed");
		allowed_to_move = false;
		action_client_ptr->cancelAllGoals();
	}

	return true;
}

bool RLLMoveIface::move_ptp(rll_msgs::MovePTP::Request &req,
			    rll_msgs::MovePTP::Response &resp)
{
	bool success;

	ROS_INFO("PTP motion requested");

	if (!manip_current_state_available())
		return false;
	manip_move_group.setStartStateToCurrentState();
	success = manip_move_group.setPoseTarget(req.pose);
	if (!success) {
		ROS_ERROR("requested pose is out of range");
		resp.success = false;
		return true;
	}

	success = run_ptp_trajectory(manip_move_group);
	if (!success)
		resp.success = false;
	else
		resp.success = true;

	return true;
}


bool RLLMoveIface::move_joints_srv(rll_msgs::MoveJoints::Request &req,
				   rll_msgs::MoveJoints::Response &resp)
{
	if (allowed_to_move) {
		move_joints(req, resp);
	} else {
		ROS_WARN("Not allowed to send move joints commands");
		resp.success = false;
		return true;
	}

	if (!resp.success) {
		ROS_FATAL("move_joints service call failed");
		allowed_to_move = false;
		action_client_ptr->cancelAllGoals();
	}

	return true;
}

bool RLLMoveIface::move_joints(rll_msgs::MoveJoints::Request &req,
			       rll_msgs::MoveJoints::Response &resp)
{
	bool success;
	std::vector<double> joints;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	ROS_INFO("Joint motion requested");

	if (!manip_current_state_available())
		return false;
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
		ROS_ERROR("requested joint values are out of range");
		resp.success = false;
		return true;
	}

	success = run_ptp_trajectory(manip_move_group);
	if (!success) {
		resp.success = false;
		return false;
	}

	resp.success = true;
	return true;
}

bool RLLMoveIface::run_ptp_trajectory(moveit::planning_interface::MoveGroupInterface &move_group)
{
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success;

	success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (!success) {
		ROS_WARN("PTP planning failed");
		return false;
	}

	success = check_trajectory(my_plan.trajectory_);
	if (!success)
		return true;

	success = modify_ptp_trajectory(my_plan.trajectory_);
	if (!success)
		return false;

	success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (!success) {
		ROS_WARN("PTP plan execution failed");
		return false;
	}

	return true;
}

bool RLLMoveIface::run_lin_trajectory(geometry_msgs::Pose goal, bool cartesian_time_parametrization)
{
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	std::vector<geometry_msgs::Pose> waypoints;
	moveit_msgs::RobotTrajectory trajectory;
	const double eef_step = 0.0005;
	const double jump_threshold = 4.5;
	bool success;

	if (!manip_current_state_available())
		return false;
	manip_move_group.setStartStateToCurrentState();
	waypoints.push_back(goal);
	double achieved = manip_move_group.computeCartesianPath(waypoints,
								eef_step, jump_threshold, trajectory);
	if (achieved < 1 && achieved > 0) {
		ROS_ERROR("only achieved to compute %f of the requested path", achieved);
		return false;
	} else if (achieved <= 0) {
		ROS_ERROR("path planning completely failed");
		return false;
	}

	success = check_trajectory(my_plan.trajectory_);
	if (!success)
		return true;

	// time parametrization happens in joint space by default
	if (cartesian_time_parametrization) {
		success = modify_lin_trajectory(trajectory);
		if (!success)
			return false;
	}

	my_plan.trajectory_= trajectory;

	success = (manip_move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if (!success) {
		ROS_ERROR("path execution failed");
		return false;
	}

	return true;
}

bool RLLMoveIface::modify_lin_trajectory(moveit_msgs::RobotTrajectory &trajectory)
{
	// Derived classes can put modifications here

	return true;
}

bool RLLMoveIface::modify_ptp_trajectory(moveit_msgs::RobotTrajectory &trajectory)
{
	// Derived classes can put modifications here

	return true;
}

bool RLLMoveIface::check_trajectory(moveit_msgs::RobotTrajectory &trajectory)
{
	if (trajectory.joint_trajectory.points.size() < 3) {
		ROS_WARN("trajectory has less than 3 points");
		return false;
	}

	std::vector<double> start = trajectory.joint_trajectory.points[0].positions;
	std::vector<double> goal = trajectory.joint_trajectory.points.back().positions;
	float distance = 0.0;
	for (int i = 0; i < start.size(); ++i)
		distance += fabs(start[i] - goal[i]);
	if (distance < 0.005) {
		ROS_INFO("trajectory: start state too close to goal state");
		return false;
	}

	return true;
}

bool RLLMoveIface::close_gripper()
{
	ROS_INFO("Closing the gripper");

	gripper_move_group.setStartStateToCurrentState();
	gripper_move_group.setNamedTarget("gripper_close");
	bool success = run_ptp_trajectory(gripper_move_group);
	if (!success)
		return false;

	return true;
}

bool RLLMoveIface::open_gripper()
{
	ROS_INFO("Opening the gripper");

	gripper_move_group.setStartStateToCurrentState();
	gripper_move_group.setNamedTarget("gripper_open");
	bool success = run_ptp_trajectory(gripper_move_group);
	if (!success)
		return false;

	return true;
}

bool RLLMoveIface::reset_to_home()
{
	if (!manip_current_state_available())
		return false;
	manip_move_group.setStartStateToCurrentState();
	manip_move_group.setNamedTarget("home_bow");
	bool success = run_ptp_trajectory(manip_move_group);
	if (!success)
		return false;

	if (!no_gripper_attached) {
		success = open_gripper();
		if (!success)
			return false;
	}

	return true;
}

bool RLLMoveIface::attach_grasp_object(std::string object_id)
{
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
	attached_object.touch_links = std::vector<std::string>{ ns + "_gripper_finger_left", ns + "_gripper_finger_right", "table" };
	planning_scene_interface.applyAttachedCollisionObject(attached_object);
}

bool RLLMoveIface::detach_grasp_object(std::string object_id)
{
	moveit_msgs::AttachedCollisionObject remove_object;

	if (object_id.empty())
		return true;

	ROS_INFO("detaching grasp object '%s'", object_id.c_str());

	std::map<std::string,moveit_msgs::AttachedCollisionObject> objects = planning_scene_interface.getAttachedObjects(std::vector<std::string>{ object_id });

	if (!objects.empty())
		remove_object = objects[object_id];
	else {
		ROS_ERROR("object not found");
		return false;
	}

	if (remove_object.object.id != object_id) {
		ROS_ERROR("The found grasp object is not the right one");
		return false;
	}

	remove_object.object.operation = remove_object.object.REMOVE;

	planning_scene_interface.applyAttachedCollisionObject(remove_object);

	moveit_msgs::CollisionObject detached_object;
	detached_object = remove_object.object;
	detached_object.operation = detached_object.ADD;
	planning_scene_interface.applyCollisionObject(detached_object);
	// TODO: figure out if this is really needed
	// occasionally, there seems to be a race condition with subsequent planning requests
	ros::Duration(0.1).sleep();
}

RLLMoveIface::~RLLMoveIface() {}

/*
 * Local Variables:
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * End:
 */
