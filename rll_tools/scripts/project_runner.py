#! /usr/bin/env python
#
# This file is part of the Robot Learning Lab SDK
#
# Copyright (C) 2019 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
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

import sys

import rospy
from rll_tools.run import idle, run_project


def main():
    server_timeout = rospy.get_param("~timeout", 5)
    auth_secret = rospy.get_param("~authentication_secret", "")
    only_idle = rospy.get_param("~only_idle", False)

    if not only_idle:
        success = run_project(server_timeout, auth_secret)
        if success is None:
            rospy.logfatal("Internal error when running project")
            sys.exit(1)
        elif not success:
            rospy.logerr("job failed")

    success = idle(server_timeout, auth_secret)
    if not success:
        rospy.logfatal("Internal error when running idle")
        sys.exit(1)


if __name__ == '__main__':
    rospy.init_node('project_runner')
    main()
