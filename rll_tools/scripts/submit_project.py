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

import rospy
import rospkg
import yaml

from os import path, remove, walk
import tarfile
import glob
import time
import requests
import json

exclude_files = []
webapp_url = "https://rll.ipr.kit.edu/"

def read_ignore_file(filename):
    with open(filename, 'r') as frp:
        file_content = frp.read().split("\n")

    # remove empty lines and comments
    file_content = filter(None, file_content)
    file_content = [ line for line in file_content if not line.startswith('#') ]

    return file_content

def tar_excludes(filename):
    if exclude_files == []:
        return False

    if filename in exclude_files:
        return True

    return False

def gen_ignore_patterns(project_path):
    gitignore_content = read_ignore_file(path.join(project_path, ".gitignore"))
    rll_submit_ignore_file = path.join(project_path, ".rll_submit_ignore")
    if path.isfile(rll_submit_ignore_file):
        rll_submit_ignore_content = read_ignore_file(rll_submit_ignore_file)
        ignore_patterns = gitignore_content + rll_submit_ignore_content
    else:
        ignore_patterns = gitignore_content

    return ignore_patterns

def gen_exclude_filelist(project_path):
    files_to_exclude = []

    ignore_patterns = gen_ignore_patterns(project_path)

    for pattern in ignore_patterns:
        for root, dirs, files in walk(project_path):
            matched_files = glob.glob(path.join(root, pattern))
            if matched_files:
                    files_to_exclude.extend(matched_files)
            for dir_name in dirs:
                matched_files = glob.glob(path.join(dir_name, pattern))
                if matched_files:
                    files_to_exclude.extend(matched_files)

    exclude_files.extend(files_to_exclude)

def create_project_archive(project_path):
    rospy.loginfo("creating project archive...")
    gen_exclude_filelist(project_path)

    submit_archive = path.join("/tmp", "rll-submit.tar.gz")
    if path.isfile(submit_archive):
        remove(submit_archive)
    with tarfile.open(submit_archive, "w:gz") as tar:
        tar.add(project_path, arcname=path.basename(project_path), exclude=tar_excludes)

    return submit_archive

def upload_archive(project_archive, api_access_cfg):
    rospy.loginfo("uploading archive...")
    submit_url = api_access_cfg["api_url"] + "jobs/submit_tar?username=" + api_access_cfg["username"] + "&project=" + project + "&token=" + api_access_cfg["token"]
    with open(project_archive) as archive:
        archive_content = archive.read()

    resp = requests.put(submit_url, data = archive_content)
    resp_msg = json.loads(resp.text)
    if resp_msg["status"] == "error":
        if resp_msg["error"] == "User has a running job in the queue":
            rospy.logerr("You have a running job in the queue for this project. You can submit again once the job has finished.")
            rospy.loginfo("You can check the job status at %sjobs", webapp_url)
        else:
            rospy.logerr("submitting project failed with error '%s'", resp_msg["error"])
    else:
        rospy.loginfo("successfully submitted job")
        rospy.loginfo("The job ID is %s", resp_msg["job_id"])
        rospy.loginfo("You can check the job status at %sjobs", webapp_url)

def submit_project():
    rospack = rospkg.RosPack()
    config_path = path.join(rospack.get_path("rll_tools"), "config", "api-access.yaml")

    if not path.isfile(config_path):
        rospy.logfatal("API access config file not found.")
        rospy.loginfo("Please download the file at %ssettings and save it to %s",
                      webapp_url, config_path)
        return

    with open(config_path, 'r') as doc:
        try:
            api_access_cfg = yaml.load(doc)
        except:
            rospy.logfatal("malformed api-access.yaml file")
            return

    try:
        project_path = rospack.get_path(project)
    except:
        rospy.logfatal("Could not find the project you want to submit. Make sure the project '%s' in your Catkin workspace", project)
        return

    time_now = time.time()

    project_archive = create_project_archive(project_path)

    rospy.loginfo("elapsed time %f", time.time() - time_now)

    upload_archive(project_archive, api_access_cfg)


if __name__ == '__main__':
    rospy.init_node("project_submitter")
    project = rospy.get_param("~project")

    submit_project()
