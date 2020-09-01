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

from typing import Union  # pylint: disable=unused-import

import actionlib
import rospy
from rll_msgs.msg import JobEnvAction, JobEnvGoal, JobStatus


# Formatting is defined in rll_move_client
# TODO: Should be refactored in the future, so it only appears once in rll_sdk

C_BOLD = '\033[1m'
C_RED = '\033[91m'
C_GREEN = '\033[92m'
C_FAIL = C_RED + C_BOLD
C_OK = C_GREEN + C_BOLD
C_END = '\033[0m'


def job_result_codes_to_string(status):
    job_codes = {
        JobStatus.SUCCESS: "success",
        JobStatus.FAILURE: "failure",
        JobStatus.INTERNAL_ERROR: "internal error"
    }
    return job_codes.get(status, "unknown")


def run_project(server_timeout=5, auth_secret="", client_ip_addr="127.0.0.1"):
    # type: (int, str, str) -> Union[bool, None]
    """
    Request the client code execution. Returns the job result as a boolean
    or None on error.
    """

    job_env = actionlib.SimpleActionClient('job_env', JobEnvAction)
    rospy.loginfo("Waiting for the 'job_env' action server.")
    rospy.sleep(0.5)

    available = job_env.wait_for_server(
        rospy.Duration.from_sec(server_timeout))

    if not available:
        rospy.logerr("The 'job_env' action server is not available! Is the "
                     "project interface running? If it is, a previous job "
                     "might still be running, try to restart the interface.")
        return None

    job_env_goal = JobEnvGoal()
    job_env_goal.authentication_secret = auth_secret
    job_env_goal.client_ip_addr = client_ip_addr

    job_env.send_goal(job_env_goal)
    rospy.loginfo("Sent 'start project' action.")

    job_env.wait_for_result()
    resp = job_env.get_result()

    if resp is None:
        rospy.logerr("The 'job_env' action server did not return a response!")
        return None

    rospy.loginfo("Executed project with status '%s'",
                  job_result_codes_to_string(resp.job.status))

    if resp.job.status_detail:
        if resp.job.status_detail.find("successfully") != -1:
            rospy.loginfo("%s%s%s", C_OK, resp.job.status_detail, C_END)
        else:
            rospy.loginfo("%s%s%s", C_FAIL, resp.job.status_detail, C_END)

    if resp.job_data:
        rospy.loginfo("Received extra job data:")
        for element in resp.job_data:
            rospy.loginfo("%s: %f", element.description, element.value)

    if resp.job.status == JobStatus.INTERNAL_ERROR:
        return None

    return resp.job.status == JobStatus.SUCCESS


def run_project_in_background(delay_seconds=0.1, **kwargs):
    """
    Start the project execution via rospy.Timer in a another thread.
    This is useful, e.g. for debugging or testing purposes.
    """

    rospy.Timer(rospy.Duration.from_sec(delay_seconds),
                lambda event: run_project(**kwargs), oneshot=True)


def idle(server_timeout=5, auth_secret=""):
    # type: (int, str) -> Union[bool, None]
    """
    Request robot and environment reset.
    """

    job_idle = actionlib.SimpleActionClient('job_idle', JobEnvAction)
    rospy.loginfo("Waiting for the 'job_idle' action server.")
    rospy.sleep(0.5)

    available = job_idle.wait_for_server(
        rospy.Duration.from_sec(server_timeout))
    if not available:
        rospy.logerr("The 'job_idle' action server is not available! "
                     "Is the project interface running?")
        return False

    job_idle_goal = JobEnvGoal()
    job_idle_goal.authentication_secret = auth_secret
    job_idle.send_goal(job_idle_goal)
    rospy.loginfo("Resetting environment back to start")

    job_idle.wait_for_result()
    resp = job_idle.get_result()

    if resp is None:
        rospy.logerr("The 'job_idle' action server did not return a response!")
        return False

    rospy.loginfo("Reset environment with status '%s'",
                  job_result_codes_to_string(resp.job.status))

    if resp.job.status == JobStatus.INTERNAL_ERROR:
        rospy.logfatal("Environment reset failed!")
        return None

    return True
