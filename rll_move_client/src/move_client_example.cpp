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

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <rll_move_client/move_client_default.h>
#include <rll_move_client/util.h>

bool execute(RLLDefaultMoveClient* const move_client)
{
  // demonstrates how to use the available services:
  //
  // 1. moveJoints(a1, a2, ..., a7) vs moveJoints(joint_values)
  // 2. movePTP(pose)
  // 3. moveLin(pose)
  // 4. moveRandom() vs moveRandom(&generated_pose)
  // 5. getCurrentPose(&current_pose)
  // 6. getCurrentJointValues(&joint_values)

  // returns true/false indicating failure, throws on critical failures
  bool success = move_client->moveJoints(0, 0, 0, -M_PI / 2, 0, 0, 0);

  if (!success)
  {
    ROS_ERROR("move_joints service call failed!");
  }

  std::vector<double> joint_values{ M_PI / 2, 0, 0, 0, 0, 0, 0 };
  move_client->moveJoints(joint_values);

  geometry_msgs::Pose goal_pose;
  goal_pose.position.x = .4;
  goal_pose.position.y = .4;
  goal_pose.position.z = .5;
  orientationFromRPY(M_PI / 2, -M_PI / 4, M_PI, &goal_pose.orientation);
  // move ptp to the specified pose
  move_client->movePTP(goal_pose);

  goal_pose.position.x = 0.2;
  goal_pose.position.y = .5;
  // linear movement to the new pose
  move_client->moveLin(goal_pose);

  // move to random pose
  move_client->moveRandom();

  // move to random position and retrieve generated pose
  move_client->moveRandom(&goal_pose);
  ROS_INFO("move_random moved to: ");
  ROS_INFO_STREAM(goal_pose);

  // get current pose, should match the previous random pose
  geometry_msgs::Pose current_pose;
  move_client->getCurrentPose(&current_pose);
  ROS_INFO("current end effector pose: ");
  ROS_INFO_STREAM(current_pose);

  // set the joint values
  std::vector<double> joint_values2{ M_PI / 2, 0.2, 0, 0, -M_PI / 4, .24, 0 };
  move_client->moveJoints(joint_values2);

  // retrieve the previously set joint values
  joint_values.clear();
  move_client->getCurrentJointValues(&joint_values);

  bool match = compareJointValues(joint_values, joint_values2);
  ROS_INFO("Set and queried joint values match: %s", match ? "yes" : "no");

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_client_example");
  RLLCallbackMoveClient<RLLDefaultMoveClient> client(&execute);
  client.spin();

  return 0;
}
