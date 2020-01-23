from os import getcwd, path
from typing import Union  # pylint: disable=unused-import

import rospy

import rospkg
import yaml
from catkin_pkg.packages import find_packages
from catkin_tools.metadata import find_enclosing_workspace

RLL_WEBAPP_URL = "https://rll.ipr.kit.edu/"


def find_project_path(project_name):
    # type: (str) -> Union[str, None]

    workspace = find_enclosing_workspace(getcwd())
    if not workspace:
        return None

    src_path = path.join(workspace, "src")
    packages = find_packages(src_path, warnings=[])
    catkin_package = [pkg_path for pkg_path, p in packages.items()
                      if p.name == project_name]
    if catkin_package:
        project_path = path.join(src_path, catkin_package[0])
    else:
        project_path = None

    return project_path


def check_filesize(project_archive, max_file_size):
    # type: (str,int) -> bool

    size = path.getsize(project_archive)
    if size > max_file_size:
        rospy.logerr("The project archive is too big for upload. "
                     "The size is %dMB and %dMB are allowed.\n"
                     "Do you have put large files into the project folder?",
                     size_in_mb(size), size_in_mb(max_file_size))
        return False

    return True


def size_in_mb(size_in_bytes):
    return size_in_bytes / (1024 * 1024)


def read_api_access_config():
    # type: () -> Union[dict, None]

    rospack = rospkg.RosPack()
    config_path = path.join(rospack.get_path("rll_tools"), "config",
                            "api-access.yaml")

    if not path.isfile(config_path):
        rospy.logfatal("API access config file not found.")
        rospy.loginfo("Please download the file at %ssettings "
                      "and save it to %s", RLL_WEBAPP_URL, config_path)
        return None

    with open(config_path, 'r') as doc:
        try:
            api_access_cfg = yaml.load(doc)
            return api_access_cfg
        except yaml.YAMLError:
            rospy.logfatal("Malformed api-access2.yaml file")

    return None
