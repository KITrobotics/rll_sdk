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

from os import path, remove, walk, environ, getcwd
import tarfile
import glob
import json
import yaml
import rospkg
import requests
from catkin_pkg.packages import find_packages
from catkin_tools.metadata import find_enclosing_workspace

import rospy


EXCLUDE_FILES = []
WEBAPP_URL = "https://rll.ipr.kit.edu/"


def read_ignore_file(filename):
    with open(filename, 'r') as frp:
        file_content = frp.read().split("\n")

    # remove empty lines and comments
    file_content = [line for line in file_content
                    if line and not line.startswith('#')]

    return file_content


def tar_excludes(filename):
    if EXCLUDE_FILES == []:
        return False

    if filename in EXCLUDE_FILES:
        return True

    return False


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

    EXCLUDE_FILES.extend(files_to_exclude)


def create_project_archive(project_path):
    rospy.loginfo("creating project archive...")
    gen_exclude_filelist(project_path)

    submit_archive = path.join("/tmp", "rll-submit.tar.gz")
    if path.isfile(submit_archive):
        remove(submit_archive)
    with tarfile.open(submit_archive, "w:gz") as tar:
        tar.add(project_path, arcname=path.basename(project_path),
                exclude=tar_excludes)

    return submit_archive


def upload_archive(project, project_archive, api_access_cfg):
    rospy.loginfo("uploading archive...")
    submit_url = (api_access_cfg["api_url"] + "jobs/submit_tar?username="
                  + api_access_cfg["username"] + "&project=" + project
                  + "&token=" + api_access_cfg["token"] + "&ros_distro="
                  + environ["ROS_DISTRO"])
    with open(project_archive) as archive:
        archive_content = archive.read()

    resp = requests.put(submit_url, data=archive_content)
    try:
        resp_msg = json.loads(resp.text)
    except ValueError:
        rospy.logfatal("failed to decode server response:\n"
                       "%s", resp.text)
        return
    if resp_msg["status"] == "error":
        if resp_msg["error"] == ("User reached max allowed limit "
                                 "of running jobs"):
            rospy.logerr("You reached the submission limit for this project. "
                         "Please wait until one of your jobs has finished.")
            rospy.loginfo("You can check the job status at %sjobs", WEBAPP_URL)
        else:
            rospy.logerr("submitting project failed with error '%s'",
                         resp_msg["error"])
    else:
        rospy.loginfo("SUCCESS: your job is submitted")
        rospy.loginfo("The job ID is %s", resp_msg["job_id"])
        rospy.loginfo("You can check the job status at %sjobs", WEBAPP_URL)


def find_project_path(project):
    workspace = find_enclosing_workspace(getcwd())
    if not workspace:
        return None

    src_path = path.join(workspace, "src")
    packages = find_packages(src_path, warnings=[])
    catkin_package = [pkg_path for pkg_path, p in packages.items()
                      if p.name == project]
    if catkin_package:
        project_path = path.join(src_path, catkin_package[0])
    else:
        project_path = None

    return project_path


def check_size(project_archive):
    max_file_size = 20 * 1024 * 1024  # upload file size is limited to 20MB

    size = path.getsize(project_archive)
    if size > max_file_size:
        rospy.logerr("The project archive is too big for upload. "
                     "The size is %dMB and %dMB are allowed.\n"
                     "Do you have put large files into the project folder?",
                     size / (1024 * 1024), max_file_size / (1024 * 1024))
        return False

    return True


def submit_project(project):
    rospack = rospkg.RosPack()
    config_path = path.join(rospack.get_path("rll_tools"), "config",
                            "api-access.yaml")

    if not path.isfile(config_path):
        rospy.logfatal("API access config file not found.")
        rospy.loginfo("Please download the file at %ssettings "
                      "and save it to %s", WEBAPP_URL, config_path)
        return

    with open(config_path, 'r') as doc:
        try:
            api_access_cfg = yaml.load(doc)
        except yaml.YAMLError:
            rospy.logfatal("malformed api-access.yaml file")
            return

    project_path = find_project_path(project)
    if not project_path:
        rospy.logfatal("Could not find the project you want to submit. "
                       "Make sure the project '%s' in your Catkin workspace.",
                       project)
        rospy.logfatal("And you need to run this submit command inside your "
                       "Catkin workspace.")
        return

    project_archive = create_project_archive(project_path)
    size_ok = check_size(project_archive)
    if not size_ok:
        return
    upload_archive(project, project_archive, api_access_cfg)


def main():
    rospy.init_node("project_submitter")
    project = rospy.get_param("~project")

    submit_project(project)


if __name__ == '__main__':
    main()
