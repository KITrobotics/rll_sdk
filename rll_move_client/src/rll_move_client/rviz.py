from typing import Union  # pylint: disable=unused-import

import rospy
from geometry_msgs.msg import Vector3, Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker
from rll_move_client.util import (linear_combine_points, pose_to_matrix)


WHITE = ColorRGBA(1, 1, 1, 1)
RED = ColorRGBA(1, 0, 0, 1)
GREEN = ColorRGBA(0, 1, 0, 1)
BLUE = ColorRGBA(0, 0, 1, 1)
YELLOW = ColorRGBA(1, 1, 0, 1)
PURPLE = ColorRGBA(1, 0, 1, 1)

TRANSPARENT_RED = ColorRGBA(1, 0, 0, .5)
TRANSPARENT_GREEN = ColorRGBA(0, 1, 0, .5)
TRANSPARENT_BLUE = ColorRGBA(0, 0, 1, .5)
TRANSPARENT_YELLOW = ColorRGBA(1, 1, 0, .5)
TRANSPARENT_ORANGE = ColorRGBA(1, .7, 0, .5)


def convert_point(point):
    if isinstance(point, Point):
        return point
    if isinstance(point, Pose):
        return point.position
    # treat it as an iterable of > 3 float tuples
    return Point(*point[:3])


def convert_color(color):
    if isinstance(color, ColorRGBA):
        return color
    if isinstance(color, (list, tuple)) and len(color) >= 4:
        return ColorRGBA(*color[:4])
    rospy.logwarn("Invalid color object: %s", color)
    return ColorRGBA(1.0, 0.0, 0.0, 1.0)


# IDEA: create a Marker class which keeps track of individual markers
# to allow updating them more easily

class RVizHelper(object):

    def __init__(self, topic=None, frame_id="world", ns="rll_marker",
                 init_sleep=.5, **kwargs):

        self._current_marker_id = 0
        self.queue_size = kwargs.get("queue_size", 100)
        self.sleep_after_publishing = init_sleep

        self.defaults = {
            "header": Header(frame_id=frame_id),
            "scale": Vector3(0.05, 0.05, 0.05),
            "color": ColorRGBA(1.0, 0.0, 0.0, 1.0),
            "ns": ns,
            "type": Marker.CUBE,
            "action": Marker.ADD,
            "lifetime": rospy.Duration(0)
        }
        self.defaults.update(kwargs)

        self._topic = topic
        self.publisher = None  # type: Union[rospy.Publisher, None]

        if topic is not None:
            self.set_topic(topic, False)

    def set_topic(self, topic, remove_all=False):
        if self.publisher is not None:
            if remove_all:
                self.remove_all()

            self.publisher.unregister()

        self._topic = topic
        self.publisher = rospy.Publisher(
            self._topic, Marker, queue_size=self.queue_size)
        rospy.sleep(self.sleep_after_publishing)

    def wait_for_subscriber(self):
        raise NotImplementedError()

    def remove_marker(self, marker):
        marker.action = Marker.DELETE
        self._publish(marker)

    def _publish(self, marker):
        if self.publisher is not None:
            self.publisher.publish(marker)
        else:
            rospy.logwarn("RvizHelper publisher is not initialized!")

    def remove_all(self):
        marker = self._get_marker_template(action=Marker.DELETEALL)
        self._publish(marker)

    def add_text(self, text, position=Point(0, 0, 1), size=.05, **kwargs):
        return self._single_marker_with_pose(
            Marker.TEXT_VIEW_FACING, text=text, position=position, size=size,
            **kwargs)

    def add_cubes(self, positions, colors=None, **kwargs):
        return self._marker_with_points_and_colors(
            Marker.CUBE_LIST, positions, colors, **kwargs)

    def add_spheres(self, positions, colors=None, **kwargs):
        return self._marker_with_points_and_colors(
            Marker.SPHERE_LIST, positions, colors, **kwargs)

    def add_sphere(self, position, size=.05, color=None, **kwargs):
        return self._single_marker_with_pose(
            Marker.SPHERE, position, size=size, color=color, **kwargs)

    def add_cube(self, pose, color=None, **kwargs):
        return self._single_marker_with_pose(
            Marker.CUBE, position=pose.position,
            orientation=pose.orientation, color=color, **kwargs)

    def add_arrow(self, start, end, size=.01, **kwargs):
        points = [start, end]

        # we keep size out of the kwargs to treat it special
        if "scale" not in kwargs:
            scale = Vector3(size, size * 2, size * 2)
            kwargs["scale"] = scale

        return self._marker_with_points_and_colors(Marker.ARROW, points,
                                                   **kwargs)

    def _marker_with_points_and_colors(
            self, marker_type, positions, colors=None, **kwargs):
        marker = self._get_marker_template(marker_type, **kwargs)
        marker.points = [convert_point(point) for point in positions]

        if colors is not None:
            marker.colors = [convert_color(color) for color in colors]

        self._publish(marker)
        return marker

    def _single_marker_with_pose(self, marker_type, position, orientation=None,
                                 **kwargs):

        marker = self._get_marker_template(marker_type, **kwargs)
        marker.pose.position = convert_point(position)

        if orientation is not None:
            marker.pose.orientation = orientation

        self._publish(marker)
        return marker

    def add_plane(self, pose, size_x=1, size_z=1, **kwargs):
        points = [
            Point(1 / 2, 0, 1 / 2),
            Point(1 / 2, 0, -1 / 2),
            Point(-1 / 2, 0, -1 / 2),
            # second triangle
            Point(-1 / 2, 0, -1 / 2),
            Point(-1 / 2, 0, 1 / 2),
            Point(1 / 2, 0, 1 / 2)
        ]
        kwargs["pose"] = pose
        kwargs["scale"] = Vector3(size_x, 1, size_z)
        return self._marker_with_points_and_colors(
            Marker.TRIANGLE_LIST, points, **kwargs)

    def add_poly_line(self, points, colors=None, **kwargs):
        return self._marker_with_points_and_colors(
            Marker.LINE_STRIP, points, colors, **kwargs)

    def add_lines(self, start_end_points, colors=None, **kwargs):
        return self._marker_with_points_and_colors(
            Marker.LINE_LIST, start_end_points, colors, **kwargs)

    def add_mesh(self, mesh_resource, pose, **kwargs):
        return self._single_marker_with_pose(
            Marker.MESH_RESOURCE, mesh_resource=mesh_resource,
            position=pose.position, orientation=pose.orientation, **kwargs)

    def _get_marker_template(self, marker_type=Marker.CUBE, **kwargs):  # noqa: E501, C901
        settings = dict(self.defaults, **kwargs)
        settings["type"] = marker_type

        # some options are treated differently or can be conveniently overriden
        if "pose" not in settings:
            settings["pose"] = Pose(Point(), Quaternion(w=1))

        # if size is in kwargs and scale is not present use size
        if "size" in kwargs and "scale" not in kwargs:
            # convert size to scale if present
            settings["scale"] = Vector3(settings["size"], settings["size"],
                                        settings["size"])

        if "orientation" in settings:
            settings["pose"].orientation = settings["orientation"]
            del settings["orientation"]

        if "color" in settings and not isinstance(settings["color"],
                                                  ColorRGBA):
            settings["color"] = convert_color(settings["color"])

        if "alpha" in settings:
            # override alpha for the global color option
            color = settings["color"]
            settings["color"] = ColorRGBA(color.r, color.g, color.b,
                                          settings["alpha"])
            del settings["alpha"]

        # if an id is given modify marker, otherwise use a new unique id
        if "update_id" in settings:
            if settings["update_id"] is not None:
                settings["id"] = settings["update_id"]
                settings["action"] = Marker.MODIFY
        else:
            settings["id"] = self._current_marker_id
            self._current_marker_id += 1

        # to avoid RViz warnings
        if marker_type == Marker.LINE_STRIP:
            settings["scale"] = Vector3(settings["scale"].x, 0, 0)
        if marker_type == Marker.TEXT_VIEW_FACING:
            settings["scale"] = Vector3(0, 0, settings["scale"].z)

        if "size" in settings:
            del settings["size"]

        if "update_id" in settings:
            del settings["update_id"]

        marker = Marker(**settings)
        marker.header.stamp = rospy.Time.now()
        return marker

    def add_pose(self, pose, size=.01):
        # type: (Pose, float) -> None
        self.add_coordinate_frame(pose_to_matrix(pose), size)

    def add_coordinate_frame(self, transform_matrix, size=.01):
        x_vec = transform_matrix[0:3, 0]
        y_vec = transform_matrix[0:3, 1]
        z_vec = transform_matrix[0:3, 2]
        pos = transform_matrix[0:3, 3]

        self.add_sphere(pos, size=size * 2, color=RED)
        self.add_arrow(pos, linear_combine_points(pos, x_vec, scale2=.1),
                       size=size, color=RED)
        self.add_arrow(pos, linear_combine_points(pos, y_vec, scale2=.1),
                       size=size, color=GREEN)
        self.add_arrow(pos, linear_combine_points(pos, z_vec, scale2=.1),
                       size=size, color=BLUE)
