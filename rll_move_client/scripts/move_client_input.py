#! /usr/bin/env python
#
# This file is part of the Robot Learning Lab Move Client
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
import atexit
import os
import rlcompleter  # noqa: F401, pylint: disable=unused-import
import readline
import termios
from math import pi

import readchar
import rospy
from geometry_msgs.msg import Pose, Point, sys
from rll_move_client.client import RLLDefaultMoveClient
from rll_move_client.formatting import override_formatting_for_ros_types
from rll_move_client.util import direction_from_quaternion, rotate_quaternion
from rll_move_client.util import orientation_from_rpy
from rll_tools.run import run_project_in_background

HISTORY = os.path.join(".rll_client_history")
DOWN = orientation_from_rpy(0, pi, 0)

try:
    readline.read_history_file(HISTORY)
except Exception:  # noqa: E722, pylint: disable=broad-except
    pass
# default history len is -1 (infinite), which may grow unruly
readline.set_history_length(2000)

atexit.register(readline.write_history_file, HISTORY)
readline.parse_and_bind('tab: complete')

# readline.set_completer_delims(' \t\n;')

CAN_READ_CHAR = False
try:
    termios.tcgetattr(sys.stdin.fileno())
    CAN_READ_CHAR = True
except:  # noqa: E722, pylint: disable=bare-except
    rospy.loginfo(
        "Cannot read single a single char, are running in a real terminal?")


def no_such_command(cmd):
    rospy.loginfo("No such command: %s", cmd)


class InputMoveClient(RLLDefaultMoveClient):
    MODE_CHAR = 1
    MODE_CMD = 2
    MOVE_LIN = "lin"
    MOVE_PTP = "ptp"

    def __init__(self):
        RLLDefaultMoveClient.__init__(self)

        self.is_input_loop_running = True
        self.mode = self.MODE_CMD
        self.motion_mode = self.MOVE_PTP
        self.move_increment = .05
        self.rot_increment = pi / 20
        self.commands = {
            "w": lambda: self.relative_motion(x=self.move_increment),
            "s": lambda: self.relative_motion(x=-self.move_increment),
            "a": lambda: self.relative_motion(y=self.move_increment),
            "d": lambda: self.relative_motion(y=-self.move_increment),
            "q": lambda: self.relative_motion(z=-self.move_increment),
            "e": lambda: self.relative_motion(z=self.move_increment),
            "c": lambda: self.get_current_cached_pose(via_cmd=True),
            "i": lambda: self.relative_motion(ro=self.rot_increment),
            "k": lambda: self.relative_motion(ro=-self.rot_increment),
            "j": lambda: self.relative_motion(pi=self.rot_increment),
            "l": lambda: self.relative_motion(pi=-self.rot_increment),
            "u": lambda: self.relative_motion(yw=self.rot_increment),
            "o": lambda: self.relative_motion(yw=-self.rot_increment),
            "h": self.home,
            "1": lambda: self.do_pick_place(gripper_open=True),
            "2": lambda: self.do_pick_place(gripper_open=False),
            "+": lambda: self.change_increment(.01),
            "-": lambda: self.change_increment(-.01),
            "m": self.toggle_relative_mode,
            "x": self.quit
        }
        self._current_pose = None

    def do_pick_place(self, gripper_open=True):
        pose_above = self.get_current_cached_pose()  # type: Pose
        z_dir = direction_from_quaternion(pose_above.orientation)
        dist = .05
        pick_point = Point(pose_above.position.x + z_dir[0] * dist,
                           pose_above.position.y + z_dir[1] * dist,
                           pose_above.position.z + z_dir[2] * dist)
        pick_pose = Pose(pick_point, pose_above.orientation)
        obj = "collision_object"  # TODO(mark): get id from somewhere
        self.pick_place(pose_above, pick_pose, pose_above, gripper_open, obj)

    def toggle_relative_mode(self):
        new_mode = self.MOVE_LIN if self.motion_mode == self.MOVE_PTP \
            else self.MOVE_PTP
        self.change_relative_mode(new_mode)

    def change_relative_mode(self, mode):
        self.motion_mode = mode
        rospy.loginfo("Changed mode to %s", self.motion_mode)

    def change_increment(self, change):
        self.move_increment += change
        self.move_increment = max(.01, self.move_increment)
        rospy.loginfo("Changed increment to %.2f", self.move_increment)

    def get_current_cached_pose(self, via_cmd=False):
        if self._current_pose is None or via_cmd:
            self._current_pose = self.get_current_pose()
            if via_cmd:
                rospy.loginfo("Current pose: %s", self._current_pose)

        return self._current_pose

    def home(self):
        self.move_joints(0, 0, 0, -pi / 2, 0, pi / 2, 0)
        self._current_pose = None  # reset

    def modify_current_pose_relative(self, **kwargs):
        current_pose = self.get_current_cached_pose()
        pose = Pose()
        pose.position.x = current_pose.position.x + kwargs.get("x", 0)
        pose.position.y = current_pose.position.y + kwargs.get("y", 0)
        pose.position.z = current_pose.position.z + kwargs.get("z", 0)

        roll = kwargs.get("ro", 0)
        pitch = kwargs.get("pi", 0)
        yaw = kwargs.get("yw", 0)
        pose.orientation = rotate_quaternion(current_pose.orientation,
                                             roll, pitch, yaw)

        return pose

    def relative_motion(self, **kwargs):
        pose = self.modify_current_pose_relative(**kwargs)

        if self.motion_mode == self.MOVE_LIN:
            resp = self.move_lin(pose)
        else:
            resp = self.move_ptp(pose)

        # update pose on success only
        if resp:
            self._current_pose = pose

    def handle_cmd_input(self, line):
        parts = line.split()
        if not parts:
            return

        if len(parts[0]) == 1:
            name = parts[0]
            cmd = self.commands.get(name, lambda: no_such_command(name))
            cmd()
        try:
            exec (line, globals())  # pylint: disable=exec-used
        except Exception as expt:  # pylint: disable=broad-except
            rospy.loginfo("Command failed: %s", expt)

    def quit(self):
        self.is_input_loop_running = False
        rospy.loginfo("Exiting loop")

    def execute(self):
        rospy.loginfo("Execute called")

        while self.is_input_loop_running:
            if not CAN_READ_CHAR or (self.mode == self.MODE_CMD):
                line = raw_input("Command >> ")
            else:
                line = readchar.readkey()

            rospy.loginfo("Read: %s", line)
            self.handle_cmd_input(line.strip())


def main():
    client = InputMoveClient()
    client.notify_job_finished()
    run_project_in_background(4)
    client.spin()


if __name__ == "__main__":
    rospy.init_node('move_client_example')
    override_formatting_for_ros_types()
    main()
