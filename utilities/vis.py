from typing import Dict

import numpy as np
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import LaserScan

__all__ = ["vis_scan", "vis_point"]

def _pub_to_topic(node: Node, topic: str, msg):
    """ Create (if needed) the publisher for the given topic on
      the node and publish the given message to said topic
    """

    if "topic_to_pub" not in globals():
        globals()["topic_to_pub"] = {}

    topic_to_pub: Dict[str, Publisher] = globals()["topic_to_pub"]
    if topic not in topic_to_pub:
        topic_to_pub[topic] = node.create_publisher(type(msg), topic, 10)

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

def vis_point(node: Node, topic: str, point: np.ndarray, frame: str):
    """Visualizes a point

    Parameters
    ----------
    node : Node
        The node that should publish the message
    topic : str
        The topic to publish the message to
    point : np.ndarray
        The point
    frame : str
        The TF frame the message should be associated with
    """
    msg = PointStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame
    msg.point.x = point[0]
    msg.point.y = point[1]
    _pub_to_topic(node, topic, msg)
