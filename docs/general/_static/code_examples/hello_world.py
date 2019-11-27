#! /usr/bin/env python
#
# This file is part of the  Robot Learning Lab Robot Playground project
#
# Copyright (C) 2019 Mark Weinreuter <uieai@student.kit.edu>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

from __future__ import print_function

from math import pi
import time

import rospy
from geometry_msgs.msg import Pose, Point

from rll_move_client.client import RLLDefaultMoveClient
from rll_move_client.util import orientation_from_rpy


def hello_world(move_client):
    print("Hello world")  # avoid using print() for logging
    rospy.loginfo("Hello ROS")  # better use rospy.loginfo(), logerror()...

    # move to a random pose, this service call requires no arguments
    move_client.move_random()

    # The robot should now be moving (in RViz)! The delays in this code
    # are only for illustrative purposes and can be removed
    time.sleep(2)

    # move all seven joints into their zero position by calling the move_joints
    # service and passing the joint values as arguments
    rospy.loginfo("calling move_joints with all joint values = 0")
    move_client.move_joints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    time.sleep(2)

    # rotate the fourth joint by 90 degrees (pi/2 since we work with radians)
    rospy.loginfo("calling move_joints with joint_4 = pi/2")
    resp = move_client.move_joints(0.0, 0.0, 0.0, pi / 2, 0.0, 0.0, 0.0)

    # previously we neglected to check the response of the service call.
    # You should always check the result of a service call. If a critical
    # failure occurs during the service call an exception will be raised.
    # If you do not check the result of service call you might miss a non
    # critical failure, because i.e. you passed an invalid pose
    if not resp:
        rospy.logwarn("move_joints service call failed (as expected)")

    # ups, moving the fourth joint by 90 degrees didn't work, we bumped into
    # the workspace boundaries
    # Let's try to move joint 2, 4 and 6 so that we end up in an upright
    # position of the end effector close to the front of the workspace.
    rospy.loginfo("calling move_joints to move into an upright position "
                  "close to the front of the workspace")
    resp = move_client.move_joints(0.0, pi / 4, 0.0, -pi / 4,
                                   0.0, -pi / 2, 0.0)

    if not resp:
        rospy.logerr("move_joints service call failed (unexpectedly)!")

    time.sleep(2)

    # moving by specifying joint angle values is not the most intuitive way
    # it's easier to specify the pose of the end effector we'd like to reach
    goal_pose = Pose()
    goal_pose.position = Point(.5, .2, .7)
    goal_pose.orientation.z = 1  # rotate 180 degrees around z (see below)

    # the move_ptp service call requires a Pose argument
    resp = move_client.move_ptp(goal_pose)

    # not all poses can be reached, remember to check the result
    if not resp:
        rospy.logerr("move_ptp service call failed")

    time.sleep(2)

    # The orientation of a pose is stored as a quaternion and usually you
    # don't specify them manually. It's easier to e.g. use euler or RPY angles
    # HINT: display the coordinate systems in RViz to visualize orientations.
    # In RViz the XYZ axes are color coded in RGB: X=red, Y=green, Z=blue
    # the end effector is pointing along the blue z-axis

    # rotate the end effector 90 degrees around the (stationary) y axis
    goal_pose.orientation = orientation_from_rpy(0, pi / 2, 0)
    rospy.loginfo("move_ptp to same position but different orientation")
    move_client.move_ptp(goal_pose)  # (error check omitted)

    time.sleep(1)

    # rotate 90deg around the y-axis and 45deg around the new (relative) x-axis
    goal_pose.orientation = orientation_from_rpy(pi / 4, pi / 2, 0)
    rospy.loginfo("move_ptp to same position but different orientation")
    move_client.move_ptp(goal_pose)  # (error check omitted)

    time.sleep(2)

    # Next up: move the end effector on a triangular path
    # while maintaining the same orientation
    rospy.loginfo("Next: move the end effector on a triangular path")

    # orient the z-axis "forward" (along the base x-axis)
    goal_pose.orientation = orientation_from_rpy(0, pi / 2, 0)

    # move to the starting position still in a ptp fashion
    goal_pose.position = Point(0.5, -0.6, 0.3)

    rospy.loginfo("move_ptp to the starting point of the triangle")
    move_client.move_ptp(goal_pose)  # (error check omitted)

    time.sleep(1)

    # move up, its a right angled triangle
    goal_pose.position.z = .7

    # this time we move on a linear trajectory to the specified pose
    rospy.loginfo("move_lin to the tip of the triangle")
    move_client.move_lin(goal_pose)  # (error check omitted)

    time.sleep(1)

    # next point is the upper right point of the triangle
    goal_pose.position.y = -0.15
    rospy.loginfo("move_lin to the upper right point of the triangle")
    move_client.move_lin(goal_pose)  # (error check omitted)

    # close the triangle by moving back diagonally to the start position
    goal_pose.position.y = -0.6
    goal_pose.position.z = .3
    rospy.loginfo("move_lin to the start to close the triangle shape")
    move_client.move_lin(goal_pose)  # (error check omitted)

    time.sleep(1)

    # Note: move_lin is not always successful, even if move_ptp succeeds.
    # This is because moving on a linear trajectory is more constraining
    # Example: move to a positive y-position will fail with move_lin
    goal_pose.position.y = 0.3
    rospy.loginfo("try to move_lin to: ")
    resp = move_client.move_lin(goal_pose)

    if not resp:
        rospy.logerr("move_lin service call failed (as expected)")

    # calling move_ptp with the exact same goal pose succeeds
    rospy.loginfo("try to move_ptp: ")
    resp = move_client.move_ptp(goal_pose)

    if not resp:
        rospy.logerr("move_ptp service call failed (unexpectedly)!")

    time.sleep(2)

    # the response sometimes holds more information than only a boolean status
    # move_random() returns the random pose the end effector moved to
    rospy.loginfo("move_random to a new random position")
    resp = move_client.move_random()  # (error check omitted)

    # we can obtain the chosen random pose from the response
    rospy.loginfo("move_random moved to: %s", resp)

    return True


def main():
    rospy.init_node('hello_world')
    client = RLLDefaultMoveClient(hello_world)
    client.spin()


if __name__ == "__main__":
    main()
