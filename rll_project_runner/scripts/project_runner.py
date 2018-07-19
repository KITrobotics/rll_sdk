#! /usr/bin/env python
# -*- coding: utf-8 -*-
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
import simplejson as json
import io
import rospkg

def job_result_codes_to_string(status):
    job_codes = {JobStatus.SUCCESS: "success", JobStatus.FAILURE: "failure",
                 JobStatus.INTERNAL_ERROR: "internal error"}
    return job_codes.get(status, "unknown")


if __name__ == '__main__':
    rospy.init_node('project_runner')

    job_env = actionlib.SimpleActionClient('job_env', JobEnvAction)
    available = job_env.wait_for_server(rospy.Duration.from_sec(2.0))
    if not available:
        rospy.logerror("job env action server is not available")
        sys.exit(1)
    rospy.sleep(0.5)
    
    job_env_goal = JobEnvGoal()
    job_env.send_goal(job_env_goal)
    rospy.loginfo("started the project")
    job_env.wait_for_result()
    resp = job_env.get_result()
    rospy.loginfo("executed project with status '%s'",
                  job_result_codes_to_string(resp.job.status))
    
    # reset robot and environment
    job_idle = actionlib.SimpleActionClient('job_idle', JobEnvAction)
    job_idle.wait_for_server(rospy.Duration.from_sec(2.0))
    job_idle_goal = JobEnvGoal()
    job_idle.send_goal(job_idle_goal)
    rospy.loginfo("resetting environment back to start")
    job_idle.wait_for_result()
    
    # get the start pose, goal pose, job status, and timing 
    start_pos_x = rospy.get_param('/iiwa/planning_iface/start_pos_x')
    start_pos_y = rospy.get_param('/iiwa/planning_iface/start_pos_y')
    start_pos_theta = rospy.get_param('/iiwa/planning_iface/start_pos_theta')
    goal_pos_x = rospy.get_param('/iiwa/planning_iface/goal_pos_x')
    goal_pos_y = rospy.get_param('/iiwa/planning_iface/goal_pos_y')
    goal_pos_theta = rospy.get_param('/iiwa/planning_iface/goal_pos_theta')
    job_status = job_result_codes_to_string(resp.job.status)
    # TODO: Get the timing from planning_iface.cpp, I will leave as constant value for now 
    plan_time = 200
    
    # generate a report
    rospy.loginfo("Generating a report")
    rospack = rospkg.RosPack() # get the path of the project solution 
    project_path = rospack.get_path('rll_planning_project') + "/report.json"
    report_data = {'start_pos': {'x': start_pos_x, 'y': start_pos_y, 'theta': start_pos_theta},
                   'goal_pos': {'x': goal_pos_x, 'y': goal_pos_y, 'theta': goal_pos_theta},
                   'job_status': job_status,
                   'plan_time' :  plan_time}                   
    with io.open(project_path, 'w', encoding='utf-8') as jsonFile:
        jsonFile.write(json.dumps(report_data, indent=4, sort_keys=True, separators=(',', ': '), ensure_ascii=False))
