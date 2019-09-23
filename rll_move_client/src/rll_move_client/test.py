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

import rospy
import unittest
import rosunit
import actionlib
from rll_msgs.msg import JobStatus, JobEnvAction, JobEnvGoal

from .util import compare_joint_values
from .error import RLLErrorCode


class TestCaseWithRLLMoveClient(unittest.TestCase):
    """
    TestCase base class that makes testing the RLLMoveClient simpler.
    """

    client = None  # expose the INITIALIZED client here

    def __init__(self, *args, **kwargs):
        super(TestCaseWithRLLMoveClient, self).__init__(*args, **kwargs)

    def assertJointValuesClose(self, joints1, joints2):
        self.assertTrue(compare_joint_values(joints1, joints2),
                        "Mismatching joint values: %s vs %s" % (
                            joints1, joints2))

    def assertServiceCallSuccess(self, resp):
        self.assertLastErrorCode(RLLErrorCode.SUCCESS)
        self.assertTrue(resp, "Service call should have succeeded")

    def assertServiceCallFailedWith(self, resp, error_code):
        self.assertLastErrorCode(error_code)
        self.assertFalse(resp, "Service call should have failed.")

    def assertLastErrorCode(self, error_code):
        last_error_code = self.client.get_last_error_code()
        self.assertEqual(last_error_code, error_code,
                         "Error codes do not match: '%s' vs '%s'! " % (
                             RLLErrorCode(last_error_code),
                             RLLErrorCode(error_code)))

    def test_0_client_available(self):
        self.assertIsNotNone(TestCaseWithRLLMoveClient.client,
                             "The static client object is not set!")


def shutdown():
    rospy.loginfo("Shutting down...")
    rospy.signal_shutdown("tests completed, force shutdown")


def generate_test_callback(package, tests, shutdown_timeout=10):
    """
    Generate a callback function that executes the given list of tests.
    Once the tests are completed the ros node is shutdown.
    """

    def execute(client):
        TestCaseWithRLLMoveClient.client = client
        for test in tests:
            test_name, test_class = test
            try:
                # catch possible errors to make sure the shutdown is called
                rosunit.unitrun(package, test_name, test_class)
            except Exception as ex:
                print(ex)

        # try to shut down graciously by giving the action client time to
        # return a response and to reset the environment
        rospy.Timer(rospy.Duration(shutdown_timeout), lambda ev: shutdown(),
                    oneshot=True)

    return execute


def run_project_in(timeout):
    rospy.Timer(rospy.Duration(timeout), lambda ev: run_project(),
                oneshot=True)


def run_project():
    # based on rll_tools run_project.py
    job_env = actionlib.SimpleActionClient('job_env', JobEnvAction)
    rospy.sleep(0.5)
    available = job_env.wait_for_server(rospy.Duration.from_sec(4.0))
    if not available:
        rospy.logerr("job env action server is not available")
        shutdown()

    job_env_goal = JobEnvGoal()
    job_env.send_goal(job_env_goal)
    rospy.loginfo("started the project")
    job_env.wait_for_result()
    resp = job_env.get_result()

    if resp is None or resp.job.status == JobStatus.INTERNAL_ERROR:
        rospy.logerr("job env action server did not return a response or an "
                     "internal error occurred!")
        shutdown()
