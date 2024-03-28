from typing import Dict, List

import numpy as np
from geometry_msgs.msg import PointStamped, Point
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

__all__ = ["vis_scan", "vis_point", "vis_points", "vis_path", "vis_lines"]

def _pub_to_topic(node: Node, topic: str, msg):
    """ Create (if needed) the publisher for the given topic on
      the node and publish the given message to said topic
    """

    if "topic_to_pub" not in globals():
        globals()["topic_to_pub"] = {}

    topic_to_pub: Dict[str, Publisher] = globals()["topic_to_pub"]
    if topic not in topic_to_pub:
        topic_to_pub[topic] = node.create_publisher(type(msg), topic, QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, depth=10))

    topic_to_pub[topic].publish(msg)

def vis_scan(node: Node, topic: str, ranges: np.ndarray, frame: str, angle_min: float, angle_increment: float, range_min: float = 0.0, range_max: float = 30.0):
    """Visualizes a range array

    Parameters
    ----------
    node : Node
        The node that should publish the message
    topic : str
        The topic to publish the message to
    ranges : np.ndarray
        The range array
    frame : str
        The TF frame the message should be associated with
    angle_min : float
        The minimum angle of the laser scan message
    angle_increment : float
        The angle increment of the laser scan message
    range_min : float, optional
        The minimum range in the ranges array, by default 0.0
    range_max : float, optional
        The maximum range in the ranges array, by default 30.0
    """

    msg = LaserScan()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame
    msg.ranges = ranges.tolist()
    msg.angle_min = angle_min
    msg.angle_increment = angle_increment
    msg.range_min = range_min
    msg.range_max = range_max
    msg.angle_max = angle_min + len(ranges) * angle_increment
    _pub_to_topic(node, topic, msg)

def _vis_points(node: Node, topic: str, points: np.ndarray, frame: str, marker_type: int, scale: float = 0.1, color: List[float] = [1.0, 0.0, 0.0, 1.0]):
    msg = Marker()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame
    msg.ns = topic
    msg.id = 0
    msg.type = marker_type
    msg.action = Marker.ADD
    msg.points = [Point(x=point[0], y=point[1]) for point in points]
    msg.pose.orientation.w = 1.0
    msg.scale.x = scale
    msg.scale.y = scale
    msg.scale.z = scale
    msg.color.r = color[0]
    msg.color.g = color[1]
    msg.color.b = color[2]
    msg.color.a = color[3]
    _pub_to_topic(node, topic, msg)


def vis_points(node: Node, topic: str, points: np.ndarray, frame: str, scale: float = 0.1, color: List[float] = [1.0, 0.0, 0.0, 1.0]):
    """Visualizes an array of points

    Parameters
    ----------
    node : Node
        The node that should publish the message
    topic : str
        The topic to publish the message to
    points : np.ndarray
        An array of points. May be of shape (2,) or (N, 2)
    frame : str
        The TF frame the message should be associated with
    scale : float, optional
        The size of the visualized points, by default 0.1
    color : List[float], optional
        The RGBA color (in range 0.0-1.0) of the visualized points, by default red
    """
    if points.ndim == 1:
        points = points[None]

    _vis_points(node, topic, points, frame, Marker.POINTS, scale, color)

@DeprecationWarning
def vis_point(node: Node, topic: str, point: np.ndarray, frame: str, scale: float = 0.1, color: List[float] = [1.0, 0.0, 0.0, 1.0]):
    """Visualizes a point

    Parameters
    ----------
    node : Node
        The node that should publish the message
    topic : str
        The topic to publish the message to
    point : np.ndarray
        The point. Shape (2,)
    frame : str
        The TF frame the message should be associated with
    scale : float, optional
        The size of the visualized points, by default 0.1
    color : List[float], optional
        The RGBA color (in range 0.0-1.0) of the visualized points, by default red
    """
    vis_points(node, topic, point, frame, scale, color)

def vis_path(node: Node, topic: str, points: np.ndarray, frame: str, scale: float = 0.1, color: List[float] = [1.0, 0.0, 0.0, 1.0]):
    """Visualizes an path of points

    Parameters
    ----------
    node : Node
        The node that should publish the message
    topic : str
        The topic to publish the message to
    points : np.ndarray
        (N, 2) array of points
    frame : str
        The TF frame the message should be associated with
    scale : float, optional
        The size of the visualized points, by default 0.1
    color : List[float], optional
        The RGBA color (in range 0.0-1.0) of the visualized points, by default red
    """
    _vis_points(node, topic, points, frame, Marker.LINE_STRIP, scale, color)

def vis_lines(node: Node, topic: str, lines: np.ndarray, frame: str, scale: float = 0.1, color: List[float] = [1.0, 0.0, 0.0, 1.0]):
    """Visualizes an array of 2D lines

    Parameters
    ----------
    node : Node
        The node that should publish the message
    topic : str
        The topic to publish the message to
    points : np.ndarray
        (N, 2, 2) array of lines. Each entry is a 2x2 matrix containing [[x1, y1], [x2, y2]]
    frame : str
        The TF frame the message should be associated with
    scale : float, optional
        The size of the visualized points, by default 0.1
    color : List[float], optional
        The RGBA color (in range 0.0-1.0) of the visualized points, by default red
    """
    _vis_points(node, topic, lines.reshape(-1, 2), frame, Marker.LINE_LIST, scale, color)
