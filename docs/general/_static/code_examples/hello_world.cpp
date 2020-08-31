/*
 * This file is part of the Robot Learning Lab Robot Playground project
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

bool helloWorld(RLLDefaultMoveClient* const move_client)
{
  std::cout << "Hello World" << std::endl;  // avoid cout for logging
  ROS_INFO("Hello ROS");                    // better use ROS_INFO, ROS_ERROR...

  // move to a random pose, this service call requires no arguments
  move_client->moveRandom();

  // The robot should now be moving (in RViz)! The delays in this code
  // are only for illustrative purposes and can be removed
  ros::Duration(2).sleep();

  // move all seven joints into their zero (initial) position
  // set all joint values of the Request to zero (unnecessary, zero is the default)
  ROS_INFO("calling move_joints with all joint values = 0");
  move_client->moveJoints(0, 0, 0, 0, 0, 0, 0);

  ros::Duration(2).sleep();

  // rotate the fourth joint by 90 degrees (pi/2 since we work with radians)
  // the remaining joint values are still equal to zero
  ROS_INFO("calling move_joints with joint_4 = pi/2");
  bool success;
  success = move_client->moveJoints(0, 0, 0, M_PI / 2, 0, 0, 0);

  // previously we neglected to check the response of the service call.
  // You should always check the result of a service call!
  if (!success)
  {
    ROS_ERROR("move_joints service call failed (as expected)");
  }

  // ups, moving the fourth joint by 90 degrees didn't work, we bumped into
  // the workspace boundaries
  // Let's try to move joint 2, 4 and 6 so that we end up in an upright position
  // of the end effector close to the front of the workspace.
  ROS_INFO("calling move_joints to move into an upright position close to the front of the workspace");
  success = move_client->moveJoints(0, M_PI / 4, 0, -M_PI / 4, 0, -M_PI / 2, 0);

  if (!success)
  {
    ROS_ERROR("move_joints service call failed (unexpectedly)");
  }

  ros::Duration(2).sleep();

  // moving by specifying joint angle values is not the most intuitive way
  // it's easier to specify the pose of the end effector we'd like to reach
  geometry_msgs::Pose goal_pose;
  goal_pose.position.x = .5;
  goal_pose.position.y = .2;
  goal_pose.position.z = .7;
  goal_pose.orientation.z = 1;  // rotate 180 degrees around z (see below)

  // the client.movePTP Request requires a Pose argument
  success = move_client->movePTP(goal_pose);

  // not all poses can be reached, remember to check the result
  if (!success)
  {
    ROS_ERROR("move_ptp service call failed");
  }

  ros::Duration(1).sleep();

  // The orientation of a pose is stored as a quaternion and its values usually
  // aren't specified manually. It's easier to use euler or RPY angles:
  // Here we rotate the end effector by 90degrees around the y axis
  // HINT: use the coordinate systems in RViz to better visualize rotations
  // in RViz the XYZ axes are color coded in RGB: X=red, Y=green, Z=blue
  // the end effector is pointing along the blue z-axis
  orientationFromRPY(0, M_PI / 2, 0, &goal_pose.orientation);

  // move to same position but different orientation!
  ROS_INFO("move_ptp to the same position but different orientation");
  move_client->movePTP(goal_pose);  // (error check omitted)

  ros::Duration(1).sleep();

  // rotate 90deg around the y-axis and 45deg around the x-axis
  orientationFromRPY(M_PI / 4, M_PI / 2, 0, &goal_pose.orientation);
  ROS_INFO("move_ptp to the same position but different orientation:");
  move_client->movePTP(goal_pose);

  ros::Duration(2).sleep();

  // You can also switch back to moving the joints directly. Let's repeat the
  // joint movement from before.
  move_client->moveJoints(0, M_PI / 4, 0, -M_PI / 4, 0, -M_PI / 2, 0);

  // Next up: move the end effector on a triangular path
  // while maintaining the same orientation
  ROS_INFO("Next: move the end effector on a triangular path");

  // orient the z-axis "forward" (along the base x-axis)
  orientationFromRPY(0, M_PI / 2, 0, &goal_pose.orientation);

  // move to the starting position still in a ptp fashion
  goal_pose.position.x = 0.3;
  goal_pose.position.y = -0.6;
  goal_pose.position.z = 0.3;

  ROS_INFO("move_ptp to the starting point of the triangle:");
  move_client->movePTP(goal_pose);

  // move up, its a right angled triangle
  goal_pose.position.z = .7;

  // this time we move on a linear trajectory to the specified pose
  ROS_INFO("moveLin to the tip of the triangle:");
  move_client->moveLin(goal_pose);

  // next point is the upper right point of the triangle
  goal_pose.position.y = -0.25;

  ROS_INFO("moveLin to the upper right point of the triangle:");
  move_client->moveLin(goal_pose);

  // close the triangle by moving back diagonally to the start position
  goal_pose.position.y = -0.6;
  goal_pose.position.z = .3;

  ROS_INFO("moveLin to the start to close the triangle shape:");
  move_client->moveLin(goal_pose);

  // note: client.moveLin is not always successful, even if client.movePTP succeeds.
  // This is because moving on a linear trajectory is more constraining
  // Example: move to a positive y-position will fail with client.moveLin
  goal_pose.position.y = 0.3;

  ROS_INFO("try to client.moveLin to:");
  success = move_client->moveLin(goal_pose);

  if (!success)
  {
    ROS_ERROR("moveLin service call failed (as expected)");
  }

  // calling movePTP with the exact same goal pose succeeds
  ROS_INFO("try to client.movePTP to:");
  success = move_client->movePTP(goal_pose);

  if (!success)
  {
    ROS_ERROR("move_ptp service call failed (unexpectedly)");
  }

  ros::Duration(2).sleep();

  // Besides moving the end effector, you can also change the arm angle to
  // move the elbow of the robot.
  double goal_arm_angle = M_PI / 2;
  move_client->moveLinArmangle(goal_pose, goal_arm_angle, true);
  // This is also possible with PTP movements.
  goal_arm_angle = -M_PI / 2;
  goal_pose.position.z = .5;
  move_client->movePTPArmangle(goal_pose, goal_arm_angle);

  // the Response object sometimes holds more information than only success
  ROS_INFO("move_random to a new random position");
  move_client->moveRandom(&goal_pose);  // (error check omitted)

  // we can obtain the chosen random pose from the response
  ROS_INFO("move_random moved to: ");
  ROS_INFO_STREAM(goal_pose);

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hello_world");
  RLLCallbackMoveClient<RLLDefaultMoveClient> client(&helloWorld);
  client.spin();

  return 0;
}
