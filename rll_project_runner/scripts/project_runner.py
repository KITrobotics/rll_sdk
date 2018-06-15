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
from rll_msgs.srv import JobEnv
from rll_msgs.msg import JobStatus

def job_result_codes_to_string(status):
    job_codes = {JobStatus.SUCCESS: "success", JobStatus.FAILURE: "failure",
                 JobStatus.INTERNAL_ERROR: "internal error"}
    return job_codes.get(status, "unknown")


if __name__ == '__main__':
    rospy.init_node('project_runner')

    try:
        job_env = rospy.ServiceProxy('job_env', JobEnv)
        resp = job_env(True)
        rospy.loginfo("executed project with status '%s'",
                      job_result_codes_to_string(resp.job.status))
    except rospy.ServiceException, e:
        rospy.loginfo("service call failed: %s", e)

    # reset robot and environment
    try:
        job_idle = rospy.ServiceProxy("job_idle", JobEnv)
        resp = job_idle(True)
    except rospy.ServiceException, e:
        rospy.loginfo("service call failed: %s", e)
