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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_iface");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(0);
	spinner.start();

	RLLMoveIface move_iface;

	actionlib::SimpleActionClient<rll_msgs::DefaultMoveIfaceAction> action_client("move_client", true);
	move_iface.action_client_ptr = &action_client;

	RLLMoveIface::JobServer server_job(nh, "job_env", boost::bind(&RLLMoveIface::run_job, &move_iface, _1, &server_job), false);
	server_job.start();
	RLLMoveIface::JobServer server_idle(nh, "job_idle", boost::bind(&RLLMoveIface::idle, &move_iface, _1, &server_idle), false);
	server_idle.start();
	ros::ServiceServer pick_place = nh.advertiseService("pick_place", &RLLMoveIface::pick_place, &move_iface);
	ros::ServiceServer move_lin = nh.advertiseService("move_lin", &RLLMoveIface::move_lin, &move_iface);
	ros::ServiceServer move_ptp = nh.advertiseService("move_ptp", &RLLMoveIface::move_ptp, &move_iface);
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
