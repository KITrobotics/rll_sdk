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

import glob
import json
import tarfile
from os import environ, path, remove, walk
from typing import List  # pylint: disable=unused-import

import requests
import rospy
from rll_tools.util import (RLL_WEBAPP_URL, check_filesize, find_project_path,
                            read_api_access_config)

SUBMISSION_TEMPLATE_URL = \
    "{0}jobs/submit_tar?username={1}&project={2}&token={3}&ros_distro={4}"

MAX_UPLOAD_FILE_SIZE = 20 * 1024 * 1024  # upload file size is limited to 20MB


def read_ignore_file(filename):
    with open(filename, 'r') as frp:
        file_content = frp.read().split("\n")

    # remove empty lines and comments
    file_content = [line for line in file_content
                    if line and not line.startswith('#')]

    return file_content


def gen_ignore_patterns(project_path):
    gitignore_file = path.join(project_path, ".gitignore")
    rll_submit_ignore_file = path.join(project_path, ".rll_submit_ignore")
    gitignore_content = None
    rll_submit_ignore_content = None

    if path.isfile(gitignore_file):
        gitignore_content = read_ignore_file(gitignore_file)
    if path.isfile(rll_submit_ignore_file):
        rll_submit_ignore_content = read_ignore_file(rll_submit_ignore_file)

    if gitignore_content and rll_submit_ignore_content:
        ignore_patterns = gitignore_content + rll_submit_ignore_content
    elif gitignore_content and not rll_submit_ignore_content:
        ignore_patterns = gitignore_content
    elif not gitignore_content and rll_submit_ignore_content:
        ignore_patterns = rll_submit_ignore_content
    else:
        ignore_patterns = []

    return ignore_patterns


def gen_exclude_filelist(project_path):
    # type: (str) -> List[str]

    files_to_exclude = []

    ignore_patterns = gen_ignore_patterns(project_path)

    for pattern in ignore_patterns:
        for root, dirs, _ in walk(project_path):
            matched_files = glob.glob(path.join(root, pattern))
            if matched_files:
                files_to_exclude.extend(matched_files)
            for dir_name in dirs:
                matched_files = glob.glob(path.join(dir_name, pattern))
                if matched_files:
                    files_to_exclude.extend(matched_files)

    return files_to_exclude


def create_project_archive(project_path):
    # type: (str) -> str

    rospy.loginfo("creating project archive...")
    files_to_exclude = gen_exclude_filelist(project_path)

    archive_path = path.join("/tmp", "rll-submit.tar.gz")
    if path.isfile(archive_path):
        remove(archive_path)

    def filter_files(tar_info):
        if tar_info.name in files_to_exclude:
            return None
        return tar_info

    with tarfile.open(archive_path, "w:gz") as tar:
        tar.add(project_path, arcname=path.basename(project_path),
                filter=filter_files)

    return archive_path


def upload_archive(project_name, tar_archive_path, api_access_cfg):
    rospy.loginfo("uploading archive...")

    url = SUBMISSION_TEMPLATE_URL.format(api_access_cfg["api_url"],
                                         api_access_cfg["username"],
                                         project_name,
                                         api_access_cfg["token"],
                                         environ["ROS_DISTRO"])

    with open(tar_archive_path, 'rb') as archive:
        archive_content = archive.read()

    resp = requests.put(url, data=archive_content)

    try:
        resp_msg = json.loads(resp.text)
    except ValueError:
        rospy.logfatal("failed to decode server response:\n"
                       "%s", resp.text)
        return False

    if resp_msg["status"] == "error":
        if resp_msg["error"] == ("User reached max allowed limit "
                                 "of running jobs"):
            rospy.logerr("You reached the submission limit for this project. "
                         "Please wait until one of your jobs has finished.")
            rospy.loginfo("You can check the job status at %sjobs",
                          RLL_WEBAPP_URL)
        else:
            rospy.logerr("submitting project failed with error '%s'",
                         resp_msg["error"])
    else:
        rospy.loginfo("SUCCESS: your job is submitted")
        rospy.loginfo("The job ID is %s", resp_msg["job_id"])
        rospy.loginfo("You can check the job status at %sjobs", RLL_WEBAPP_URL)
        return True

    return False


def submit_project(project_name):
    # type: (str) -> bool

    api_access_cfg = read_api_access_config()
    if api_access_cfg is None:
        return False

    project_path = find_project_path(project_name)
    if not project_path:
        rospy.logfatal("Could not find the project you want to submit. "
                       "Make sure the project '%s' in your Catkin workspace.",
                       project_name)
        rospy.logfatal("And you need to run this submit command inside your "
                       "Catkin workspace.")
        return False

    archive_path = create_project_archive(project_path)
    size_ok = check_filesize(archive_path, MAX_UPLOAD_FILE_SIZE)
    if not size_ok:
        return False

    return upload_archive(project_name, archive_path,
                          api_access_cfg)
