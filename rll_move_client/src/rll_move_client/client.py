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
import collections

import actionlib
import rospy
from rll_msgs.msg import DefaultMoveIfaceAction
from rll_msgs.srv import MoveJoints, MovePTP, MoveLin, MoveRandom, \
    GetJointValues, GetPose, PickPlace

from .error import ServiceCallFailure, CriticalServiceCallFailure, RLLErrorCode
from .util import apply_ansi_format, C_NAME, C_END, C_OK, C_FAIL, C_WARN, \
    C_INFO


class RLLMoveClientBase(object):

    def __init__(self):
        self.verbose = True
        self._exception_on_any_failure = False
        self._last_error_code = RLLErrorCode()

    def set_exception_on_any_failure(self, raise_exception=True):
        self._exception_on_any_failure = raise_exception

    def get_last_error_code(self):
        return self._last_error_code

    def _call_service_with_error_check(self, srv, srv_name, log_format,
                                       handle_response_func, *args):

        self._log_service_call(srv_name, log_format, *args)

        resp = srv(*args)
        self._last_error_code = resp.error_code
        return handle_response_func(srv_name, resp)

    def _log_service_call(self, srv_name, log_format="%s requested", *args):
        if self.verbose:
            name = apply_ansi_format(srv_name, C_NAME)
            if len(args) > 0:
                rospy.loginfo(log_format, name, args)
            else:
                rospy.loginfo(log_format, name)

    def _handle_response_error_code(self, srv_name, resp):
        name = apply_ansi_format(srv_name, C_NAME)

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


class RLLActionMoveClient:

    def __init__(self, execute_func=None, action_name="move_client"):
        # TODO(uieai) do we need to inherit from RLLMoveClientBase ?
        self.execute_func = execute_func

        self.server = actionlib.SimpleActionServer(
            action_name, DefaultMoveIfaceAction, self.__execute, False)

        rospy.loginfo("Action server started")
        self.server.start()

    def __execute(self, req):

        rospy.loginfo("Action triggered")

        try:
            if self.execute_func is not None:
                result = self.execute_func(self)
            else:
                result = self.execute()

            # if the callback returns not None treat it as a success indication
            if result is not None:
                if result:
                    rospy.loginfo("Callback completed %ssuccessfully%s",
                                  C_OK, C_END)
                else:
                    rospy.loginfo("Callback completed %sunsuccessfully%s",
                                  C_WARN, C_END)

            self.server.set_succeeded()

        except ServiceCallFailure as expt:
            rospy.logerr("The action routine was interrupted by an uncaught "
                         "exception: \n%s", expt)
            self.server.set_aborted()

        rospy.loginfo("Action completed")

    def execute(self):
        raise NotImplementedError(
            "You must override the 'execute() method!")


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
        self.move_joints_service = rospy.ServiceProxy(
            self.MOVE_JOINTS_SRV_NAME, MoveJoints)
        self.move_ptp_service = rospy.ServiceProxy(
            self.MOVE_PTP_SRV_NAME, MovePTP)
        self.move_lin_service = rospy.ServiceProxy(
            self.MOVE_LIN_SRV_NAME, MoveLin)
        self.move_random_service = rospy.ServiceProxy(
            self.MOVE_RANDOM_SRV_NAME, MoveRandom)

        # available 'getter' -> return values
        self.get_current_pose_service = rospy.ServiceProxy(
            self.GET_POSE_SRV_NAME, GetPose)
        self.get_current_joint_values_service = rospy.ServiceProxy(
            self.GET_JOINT_VALUES_SRV_NAME, GetJointValues)

    def move_ptp(self, pose):
        return self._call_service_with_error_check(
            self.move_ptp_service, self.MOVE_PTP_SRV_NAME,
            "%s requested with: %s", self._handle_response_error_code, pose)

    def move_joints(self, *args):
        # either an iterable or seven joint values
        if len(args) == 1 and (
                isinstance(args, collections.Iterable) and len(args[0]) == 7):
            args = args[0]
        elif len(args) != 7:
            rospy.logwarn("You need pass seven joint values, either as "
                          "an iterable or as seven separate arguments")
            return False

        return self._call_service_with_error_check(
            self.move_joints_service, self.MOVE_JOINTS_SRV_NAME,
            "%s requested with: %s", self._handle_response_error_code,
            *args)

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

    def move_random(self):

        return self._call_service_with_error_check(
            self.move_random_service, self.MOVE_RANDOM_SRV_NAME,
            "%s requested",
            self._handle_resp_with_values(lambda resp: resp.pose))

    def move_lin(self, pose):

        return self._call_service_with_error_check(
            self.move_lin_service, self.MOVE_LIN_SRV_NAME,
            "%s requested with %s", self._handle_response_error_code,
            pose)

    def get_current_joint_values(self):
        def handle_joint_values(resp):
            return [resp.joint_1, resp.joint_2, resp.joint_3, resp.joint_4,
                    resp.joint_5, resp.joint_6, resp.joint_7]

        return self._call_service_with_error_check(
            self.get_current_joint_values_service,
            self.GET_JOINT_VALUES_SRV_NAME,
            "%s requested", self._handle_resp_with_values(handle_joint_values))

    def get_current_pose(self):
        return self._call_service_with_error_check(
            self.get_current_pose_service, self.GET_POSE_SRV_NAME,
            "%s requested",
            self._handle_resp_with_values(lambda resp: resp.pose))


class PickPlaceClient(RLLMoveClientBase):  # TODO(uieai) test pick place

    PICK_PLACE_SRV_NAME = "pick_place"

    def __init__(self):
        RLLMoveClientBase.__init__(self)

        self.pick_place_service = rospy.ServiceProxy(
            self.PICK_PLACE_SRV_NAME, PickPlace)

    def pick_place(self, pose_above, pose_grip, gripper_close,
                   grasp_object):
        return self._call_service_with_error_check(
            self.pick_place_service, self.PICK_PLACE_SRV_NAME,
            "%s requested with: %s", self._handle_response_error_code,
            pose_above, pose_grip, gripper_close, grasp_object)


class RLLDefaultMoveClient(RLLActionMoveClient, RLLBasicMoveClient,
                           PickPlaceClient):

    def __init__(self, execute_func=None, action_name="move_client"):
        RLLActionMoveClient.__init__(self, execute_func, action_name)
        RLLBasicMoveClient.__init__(self)
        PickPlaceClient.__init__(self)
