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
from rll_move_client.error import RLLErrorCode
from rll_move_client.util import compare_joint_values
from rll_tools.run import run_project


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


def run_project_or_shutdown():
    job_success = run_project()
    # on failure shutdown
    if not job_success:
        shutdown()
