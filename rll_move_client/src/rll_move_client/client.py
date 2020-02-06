#! /usr/bin/env python
#
# This file is part of the Robot Learning Lab Move Client
#
# Copyright (C) 2019 Mark Weinreuter <uieai@student.kit.edu>
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
import re
import socket
import traceback
from math import pi
from typing import Union, Sequence, List  # pylint: disable=unused-import

import rospy
from geometry_msgs.msg import Pose, Point
from std_srvs.srv import SetBool, SetBoolRequest
from rll_move_client.error import (ServiceCallFailure,
                                   CriticalServiceCallFailure, RLLErrorCode)
from rll_move_client.formatting import (ansi_format, C_NAME, C_END, C_OK,
                                        C_FAIL, C_WARN, C_INFO,
                                        get_exception_raising_function,
                                        override_formatting_for_ros_types)
from rll_move_client.util import orientation_from_rpy
from rll_msgs.srv import (MoveJoints, MovePTP, MoveLin, MoveRandom, GetPose,
                          GetJointValues, PickPlace)


class RLLMoveClientBase(object):

    def __init__(self):
        self.verbose = True
        self._exception_on_any_failure = False
        self._last_error_code = RLLErrorCode()
        self._logging_stream = rospy.logdebug

    def set_exception_on_any_failure(self, raise_exception=True):
        # type: (bool) -> None

        self._exception_on_any_failure = raise_exception

    def get_last_error_code(self):
        # () -> RLLErrorCode
        return self._last_error_code

    def _call_service_with_error_check(self, srv, srv_name, log_format,
                                       handle_response_func, *args):

        self._log_service_call(srv_name, log_format, *args)
        try:
            resp = srv(*args)
        except rospy.ServiceException as exp:
            # Retrieve the current stack
            trace = "".join(traceback.format_stack()[:-1])
            raise ServiceCallFailure(
                RLLErrorCode.SERVICE_CALL_CLIENT_ERROR,
                "Failed to call %s service: %s\n%s" % (srv_name, exp, trace))

        self._last_error_code = resp.error_code
        return handle_response_func(srv_name, resp)

    def _log_service_call(self, srv_name, log_format, *args):
        if self.verbose:
            name = ansi_format(srv_name, C_NAME)
            if args:
                rospy.loginfo(log_format, name, *args)
            else:
                rospy.loginfo(log_format, name)

    def _handle_response_error_code(self, srv_name, resp):
        name = ansi_format(srv_name, C_NAME)

        if resp.success:
            if self.verbose:
                rospy.loginfo("%s %ssucceeded%s", name, C_OK, C_END)
            return True

        error_code = RLLErrorCode(resp.error_code)

        if error_code.is_critical_failure():
            rospy.logerr(
                "%s %sfailed critically with error: %s%s",
                name, C_FAIL, C_END, error_code)

            raise CriticalServiceCallFailure(
                error_code,
                "Service call %s failed with error: %s" % (
                    srv_name, error_code))
        else:
            rospy.logwarn("%s %sfailed: %s%s", name, C_FAIL, C_END, error_code)

            if self._exception_on_any_failure:
                raise ServiceCallFailure(
                    error_code,
                    "Service call %s failed: %s" % (srv_name, error_code))
            elif error_code.code == RLLErrorCode.JOB_EXECUTION_TIMED_OUT:
                raise ServiceCallFailure(
                    error_code,
                    "You exceeded the maximum job execution duration.")

        hint = error_code.get_hint()
        if hint is not None:
            rospy.loginfo("%sPossible failure reason:%s %s", C_INFO, C_END,
                          hint)

        return False

    def _handle_resp_with_values(self, resp_handler_func):
        def handle(name, resp):
            success = self._handle_response_error_code(name, resp)

            # some service calls can return additional values
            # therefore, swap the result, in such a fashion that the still
            # can be evaluated as truthy/falsey
            if success:
                return resp_handler_func(resp)

            return None

        return handle


class RLLMoveClientListener(object):
    JOB_FINISHED_SRV_NAME = "job_finished"

    def __init__(self, execute_func=None, tcp_port=5005):
        self.execute_func = execute_func
        self._is_job_running = False

        # basic check that we are running in the /iiwa or /sim namespace
        # there may or may not be a leading and trailing slash
        # and on the real robots the ns will have a _<number> suffix
        ns = rospy.get_namespace().strip("/")
        ns_regex = r"^(iiwa(_\d)?|sim(_\d)?)$"

        if not bool(re.match(ns_regex, ns)):
            rospy.logerr("You are running your code in the namespace '%s' "
                         "Is your environment set up correctly?", ns)

        self._job_finished_service = rospy.ServiceProxy(
            self.JOB_FINISHED_SRV_NAME, SetBool)

        # init TCP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(2.0)
        # listen on all interfaces
        self.sock.bind(('', tcp_port))
        # only allow one connection
        self.sock.listen(1)

        rospy.loginfo("Socket listener started")

    def execute(self):  # pylint: disable=no-self-use
        # type: () -> bool

        rospy.logerr("No client code found. "
                     "You must pass an execute() method to the client "
                     "or overwrite the execute() method of the client!")
        return False

    def __execute(self):
        self._is_job_running = True
        success = None
        rospy.loginfo("Code execution triggered")

        try:
            if self.execute_func is not None:
                success = self.execute_func(self)
            else:
                success = self.execute()
        except ServiceCallFailure as expt:
            rospy.logerr("The client routine was interrupted by an uncaught "
                         "service call exception: \n%s", expt)
        except NotImplementedError:
            rospy.logerr("You haven't (fully) implemented function: %s",
                         ansi_format(get_exception_raising_function(), C_NAME))
        except KeyboardInterrupt:
            rospy.logwarn("The client routine was interrupted by the user.")

        except Exception:  # pylint: disable=broad-except
            # catch all exceptions to ensure that the interface is informed
            # about the job result, but still log the full stack trace
            rospy.logerr("The client routine was interrupted by an uncaught "
                         "exception: %s", traceback.format_exc())

        if not isinstance(success, bool):
            rospy.loginfo("Client code did not return boolean indicating "
                          "success, assuming %sunsuccessful completion%s",
                          C_WARN, C_END)
            success = False
        elif success:
            rospy.loginfo("Client code completed %ssuccessfully%s",
                          C_OK, C_END)
        else:
            rospy.loginfo("Client code completed %sunsuccessfully%s",
                          C_WARN, C_END)

        self.notify_job_finished(success)
        rospy.loginfo("Code execution completed")

        return success

    def notify_job_finished(self, success=False, force_run=False):
        if not self._is_job_running and not force_run:
            # only set the result if a job is actually running
            return None

        self._is_job_running = False

        req = SetBoolRequest(data=success)
        try:
            resp = self._job_finished_service(req)
        except rospy.service.ServiceException:
            rospy.logwarn("Failed to call job_finished: %s",
                          traceback.format_exc())
            return None

        if not resp.success:
            rospy.logerr("failed to report execution result to interface")

        return resp

    def _on_ros_shutdown(self):
        rospy.logdebug("ROS shutdown triggered")
        if self._is_job_running:
            rospy.logwarn("Client code interrupted during a job run!")
            self.notify_job_finished(False)

    def spin(self):
        # type: () -> None

        buffer_size = 10

        rospy.on_shutdown(self._on_ros_shutdown)

        while not rospy.is_shutdown():
            try:
                conn, addr = self.sock.accept()
            except socket.timeout:
                continue
            except socket.error as err:
                rospy.loginfo("socket error %s", err)
                continue

            rospy.loginfo("got a connection from addr %s", addr)

            try:
                data = conn.recv(buffer_size)  # pylint: disable=no-member
            except socket.error as err:
                rospy.logerr("socket error %s", err)
                continue

            if not data:
                rospy.logerr("did not receive any data in listener spinner")
                continue

            if data == b"start":
                rospy.loginfo("received start signal")
                conn.send(b'ok')  # pylint: disable=no-member
                self.__execute()

            else:
                rospy.logerr("error receiving start signal")
                conn.send(b'error')  # pylint: disable=no-member


class RLLBasicMoveClient(RLLMoveClientBase):
    MOVE_LIN_SRV_NAME = "move_lin"
    MOVE_JOINTS_SRV_NAME = "move_joints"
    MOVE_PTP_SRV_NAME = "move_ptp"
    MOVE_RANDOM_SRV_NAME = "move_random"
    GET_POSE_SRV_NAME = "get_current_pose"
    GET_JOINT_VALUES_SRV_NAME = "get_current_joint_values"

    def __init__(self):
        RLLMoveClientBase.__init__(self)

        # available movement services -> return error_code
        self._move_joints_service = rospy.ServiceProxy(
            self.MOVE_JOINTS_SRV_NAME, MoveJoints)
        self._move_ptp_service = rospy.ServiceProxy(
            self.MOVE_PTP_SRV_NAME, MovePTP)
        self._move_lin_service = rospy.ServiceProxy(
            self.MOVE_LIN_SRV_NAME, MoveLin)
        self._move_random_service = rospy.ServiceProxy(
            self.MOVE_RANDOM_SRV_NAME, MoveRandom)

        # available 'getter' -> return values
        self._get_current_pose_service = rospy.ServiceProxy(
            self.GET_POSE_SRV_NAME, GetPose)
        # pylint: disable=invalid-name
        self.get_current_joint_values_service = rospy.ServiceProxy(
            self.GET_JOINT_VALUES_SRV_NAME, GetJointValues)

    def move_to_home_position(self, move_ptp=False):
        if not move_ptp:
            return self.move_joints(0, 0, 0, -pi / 2, 0, pi / 2, 0)

        return self.move_ptp(Pose(Point(0.20, 0.00, 0.39),
                                  orientation_from_rpy(3.14, 0.00, 3.14)))

    def move_ptp(self, pose):
        # type: (Pose) -> bool

        return self._call_service_with_error_check(
            self._move_ptp_service, self.MOVE_PTP_SRV_NAME,
            "%s requested with: %s", self._handle_response_error_code, pose)

    def move_joints(self, *args):
        # type: (Union[Sequence[float], float]) -> bool

        # either an iterable or seven joint values
        if len(args) == 1 and (
                isinstance(args, (tuple, list)) and len(args[0]) == 7):
            args = args[0]
        elif len(args) != 7:
            rospy.logwarn("You need pass seven joint values, either as "
                          "an iterable or as seven separate arguments")
            return False

        return self._call_service_with_error_check(
            self._move_joints_service, self.MOVE_JOINTS_SRV_NAME,
            "%s requested with: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
            self._handle_response_error_code, *args)

    def move_random(self):
        # type: () -> Union[Pose, None]

        return self._call_service_with_error_check(
            self._move_random_service, self.MOVE_RANDOM_SRV_NAME,
            "%s requested",
            self._handle_resp_with_values(lambda resp: resp.pose))

    def move_lin(self, pose):
        # type: (Pose) -> bool

        return self._call_service_with_error_check(
            self._move_lin_service, self.MOVE_LIN_SRV_NAME,
            "%s requested with %s", self._handle_response_error_code,
            pose)

    def get_current_joint_values(self):
        # type: () -> Union[List[float], None]

        def handle_joint_values(resp):
            return [resp.joint_1, resp.joint_2, resp.joint_3, resp.joint_4,
                    resp.joint_5, resp.joint_6, resp.joint_7]

        return self._call_service_with_error_check(
            self.get_current_joint_values_service,
            self.GET_JOINT_VALUES_SRV_NAME,
            "%s requested", self._handle_resp_with_values(handle_joint_values))

    def get_current_pose(self):
        # type: () -> Union[Pose, None]

        return self._call_service_with_error_check(
            self._get_current_pose_service, self.GET_POSE_SRV_NAME,
            "%s requested",
            self._handle_resp_with_values(lambda resp: resp.pose))


class PickPlaceClient(RLLMoveClientBase):  # TODO(uieai) test pick place

    PICK_PLACE_SRV_NAME = "pick_place"

    def __init__(self):
        RLLMoveClientBase.__init__(self)

        self._pick_place_service = rospy.ServiceProxy(
            self.PICK_PLACE_SRV_NAME, PickPlace)

    def pick_place(self, pose_above, pose_grip,  # pylint: disable=r0913
                   gripper_close, grasp_object):
        # type: (Pose, Pose, bool, str) -> bool

        return self._call_service_with_error_check(
            self._pick_place_service, self.PICK_PLACE_SRV_NAME,
            "%s requested with: %s->%s close:%s, grasp:%s",
            self._handle_response_error_code,
            pose_above, pose_grip, gripper_close,
            grasp_object)


class RLLDefaultMoveClient(RLLMoveClientListener, RLLBasicMoveClient,
                           PickPlaceClient):

    def __init__(self, execute_func=None):
        RLLMoveClientListener.__init__(self, execute_func)
        RLLBasicMoveClient.__init__(self)
        PickPlaceClient.__init__(self)

        override_formatting_for_ros_types()
