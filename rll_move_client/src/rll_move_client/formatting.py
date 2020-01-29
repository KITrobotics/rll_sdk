import sys
import traceback

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion, Point
from rll_move_client.util import quaternion_to_rpy

C_BOLD = '\033[1m'
C_BLUE = '\033[94m'
C_NAME = C_BLUE + C_BOLD
C_GREEN = '\033[92m'
C_YELLOW = '\033[93m'
C_RED = '\033[91m'
C_MAGENTA = '\033[95m'
C_OK = C_GREEN + C_BOLD
C_INFO = C_YELLOW + C_BOLD
C_WARN = C_MAGENTA + C_BOLD
C_FAIL = C_RED + C_BOLD
C_END = '\033[0m'


def ansi_format(text, ansi_code=C_NAME):
    return "{0}{1}{2}".format(ansi_code, text, C_END)


def color_blue(text):
    return ansi_format(text, C_BLUE)


def color_failure(text):
    return ansi_format(text, C_RED + C_BOLD)


def color_success(text, success=True):
    if success:
        return ansi_format(text, C_GREEN + C_BOLD)

    return ansi_format(text, C_RED + C_BOLD)


def pass_fail(success):
    return color_success("Pass" if success else "Fail", success)


def bold_text(text):
    return ansi_format(text, C_BOLD)


def get_color(success):
    return C_GREEN if success else C_RED


def format_pose(pose):
    # type: (Pose) -> str
    return format_point(pose.position) + ", " + format_quaternion(
        pose.orientation)


def format_quaternion(quat):
    # type: (Quaternion) -> str
    roll, pitch, yaw = quaternion_to_rpy(quat)
    return "rpy(%.2f, %.2f, %.2f)" % (roll, pitch, yaw)


def format_point(point):
    # type: (Point) -> str
    return "Point(%.2f, %.2f, %.2f)" % (point.x, point.y, point.z)


def format_list(_list, max_n=20):
    text = "["
    for val in _list[:max_n]:
        text += str(val) + ", "
    return text + "]"


def format_joint_values(joint_values):
    return "[" + ", ".join(["%.2f" % v for v in joint_values]) + "]"


def override_formatting_for_ros_types():
    """
    Change the way rospy.log* outputs Point, Quaternion and Pose objects.
    E.g. this outputs Quaternions as rpy(...) which is probably more useful.
    """
    Pose.__str__ = format_pose
    Pose.__repr__ = format_pose
    Point.__str__ = format_point
    Point.__repr__ = format_point
    Quaternion.__str__ = format_quaternion
    Quaternion.__repr__ = format_quaternion


def get_exception_raising_function():
    try:
        trace = sys.exc_info()[-1]
        stk = traceback.extract_tb(trace, -1)
        fname = stk[0][2]
        return fname
    except Exception:  # noqa: E731, pylint: disable=broad-except
        return "Unknown function"
