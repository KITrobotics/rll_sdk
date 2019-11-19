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

from __future__ import print_function
import time
import unittest

import rospy
import rosunit
import actionlib
from rll_msgs.msg import JobStatus, JobEnvAction, JobEnvGoal

from rll_move_client.util import compare_joint_values
from rll_move_client.error import RLLErrorCode


class TestCaseWithRLLMoveClient(unittest.TestCase):
    """
    TestCase base class that makes testing the RLLMoveClient simpler.
    """

    client = None  # expose the INITIALIZED client here

    def __init__(self, *args, **kwargs):
        super(TestCaseWithRLLMoveClient, self).__init__(*args, **kwargs)

    def assert_joint_values_lose(self, joints1, joints2):
        self.assertTrue(compare_joint_values(joints1, joints2),
                        "Mismatching joint values: %s vs %s" % (
                            joints1, joints2))

    def assert_last_srv_call_success(self, resp):
        self.assert_last_error_code(RLLErrorCode.SUCCESS)
        self.assertTrue(resp, "Service call should have succeeded")

    def assert_last_srv_call_failed(self, resp, error_code):
        self.assert_last_error_code(error_code)
        self.assertFalse(resp, "Service call should have failed.")

    def assert_last_error_code(self, error_code):
        last_error_code = self.client.get_last_error_code()
        self.assert_error_code_equals(error_code, last_error_code)

    def assert_error_code_equals(self, code1, code2):
        self.assertEqual(code1, code2,
                         "Error codes do not match: '%s' vs '%s'! " % (
                             RLLErrorCode(code1), RLLErrorCode(code2)))

    def test_0_client_available(self):
        self.assertIsNotNone(TestCaseWithRLLMoveClient.client,
                             "The static client object is not set!")


def concurrent_call(thread_func, count=10, delay_between_starts=0):
    """
    Run the given function n times concurrently. The function receives the
    index (the i-th invocation) as an argument. The first thread gets a
    head start in case you try to call the same service, etc.

    Returns a ordered list of the return values.

    :param thread_func:
    :param n:
    :param delay_between_starts:
    :return:
    """
    import threading
    lock = threading.Lock()
    results = [None for _ in range(count)]

    def collect(index):
        result = None
        try:
            result = thread_func(index)
        except StandardError as ex:
            rospy.logerr("Exception in thread execution: %s", ex)

        lock.acquire()
        try:
            rospy.loginfo("collected thread #%d", index)
            results[index] = result
        finally:
            lock.release()

    threads = [threading.Thread(target=collect, args=(i,))
               for i in range(count)]

    threads[0].start()
    time.sleep(.5)  # give it a bit of an head start to make sure it is running

    for thread in threads[1:]:
        thread.start()
        time.sleep(delay_between_starts)

    for thread in threads:
        thread.join()

    return results


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
            except StandardError as ex:
                print(ex)

        if shutdown_timeout > 0:
            # try to shut down graciously by giving the action client time to
            # return a response and to reset the environment
            rospy.Timer(rospy.Duration(shutdown_timeout),
                        lambda ev: shutdown(), oneshot=True)

    return execute


def run_project_in(timeout):
    rospy.Timer(rospy.Duration(timeout), lambda ev: run_project_or_shutdown(),
                oneshot=True)


def run_project_or_shutdown():
    job_success, _ = run_project()
    # on failure shutdown
    if not job_success:
        shutdown()


def run_project():
    # based on rll_tools run_project.py
    job_env = actionlib.SimpleActionClient('job_env', JobEnvAction)

    available = job_env.wait_for_server(rospy.Duration.from_sec(4.0))
    if not available:
        rospy.logerr("job env action server is not available")
        return False, -1

    # TODO: move the project_runner from rll_tools into a module,
    #       so we can reuse the ocde here and can get rid of the
    #       duplicate code
    # note: all checks have to be disabled because of
    #       https://github.com/PyCQA/pylint/issues/214
    # pylint: disable=all
    job_env_goal = JobEnvGoal()
    job_env_goal.client_ip_addr = "127.0.0.1"
    job_env.send_goal(job_env_goal)
    rospy.loginfo("started the project")
    job_env.wait_for_result()
    resp = job_env.get_result()
    # pylint: enable=all

    state = job_env.get_state()
    rospy.loginfo("Goal state: %s", state)

    if resp is None or resp.job.status == JobStatus.INTERNAL_ERROR:
        rospy.logerr("job env action server did not return a response or an "
                     "internal error occurred!")
        return False, state

    rospy.loginfo("Job completed")
    return True, state


def idle():
    # based on rll_tools run_project.py
    job_idle = actionlib.SimpleActionClient('job_idle', JobEnvAction)
    available = job_idle.wait_for_server(rospy.Duration.from_sec(4.0))
    if not available:
        rospy.logerr("job idle action server is not available")
        return False, -1

    job_idle_goal = JobEnvGoal()
    job_idle.send_goal(job_idle_goal)
    rospy.loginfo("resetting environment back to start")
    job_idle.wait_for_result()
    resp = job_idle.get_result()

    state = job_idle.get_state()
    rospy.loginfo("Goal state: %s", state)

    if resp.job.status == JobStatus.INTERNAL_ERROR:
        rospy.logfatal("environment reset failed")
        return False, state

    rospy.loginfo("Idle completed")
    return True, state
