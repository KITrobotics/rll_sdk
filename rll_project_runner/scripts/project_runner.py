#! /usr/bin/env python
#
# This file is part of the Robot Learning Lab SDK
#
# Copyright (C) 2018 Wolfgang Wiedmeyer <wolfgang.wiedmeyer@kit.edu>
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

import rospy
import actionlib
from rll_msgs.msg import *
import sys

def job_result_codes_to_string(status):
    job_codes = {JobStatus.SUCCESS: "success", JobStatus.FAILURE: "failure",
                 JobStatus.INTERNAL_ERROR: "internal error"}
    return job_codes.get(status, "unknown")

# reset robot and environment
def idle():
    job_idle = actionlib.SimpleActionClient('job_idle', JobEnvAction)
    rospy.sleep(0.5)
    available = job_idle.wait_for_server(rospy.Duration.from_sec(4.0))
    if not available:
        rospy.logerr("job idle action server is not available")
        sys.exit(1)

    job_idle_goal = JobEnvGoal()
    job_idle.send_goal(job_idle_goal)
    rospy.loginfo("resetting environment back to start")
    job_idle.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('project_runner')
    only_idle = rospy.get_param("~only_idle")
    if only_idle:
        idle()
        sys.exit(0)

    job_env = actionlib.SimpleActionClient('job_env', JobEnvAction)
    rospy.sleep(0.5)
    available = job_env.wait_for_server(rospy.Duration.from_sec(4.0))
    if not available:
        rospy.logerr("job env action server is not available")
        sys.exit(1)

    job_env_goal = JobEnvGoal()
    job_env.send_goal(job_env_goal)
    rospy.loginfo("started the project")
    job_env.wait_for_result()
    resp = job_env.get_result()
    rospy.loginfo("executed project with status '%s'",
                  job_result_codes_to_string(resp.job.status))
    if resp.job_data:
        rospy.loginfo("extra job data:")
        for element in resp.job_data:
            rospy.loginfo("%s: %f", element.description, element.value)

    idle()
