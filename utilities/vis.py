from rclpy.node import Node
from rclpy.publisher import Publisher

import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from typing import Dict

__all__ = ["vis_pub_for_topic", "vis_scan"]

def vis_pub_for_topic(node: Node, topic: str) -> Publisher:
    if "topic_to_pub" not in globals():
        globals()["topic_to_pub"] = {}

    topic_to_pub: Dict[str, Publisher] = globals()["topic_to_pub"]
    if topic not in topic_to_pub:
        topic_to_pub[topic] = node.create_publisher(LaserScan, topic, 10)

    return topic_to_pub[topic]

def vis_scan(node: Node, topic: str, ranges: np.ndarray, frame: str, angle_min: float, angle_increment: float, range_min: float = 0.0, range_max: float = 30.0):

    msg = LaserScan()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame
    msg.ranges = ranges.tolist()
    msg.angle_min = angle_min
    msg.angle_increment = angle_increment
    msg.range_min = range_min
    msg.range_max = range_max
    msg.angle_max = angle_min + len(ranges) * angle_increment
    vis_pub_for_topic(node, topic).publish(msg)

def vis_point(node: Node, topic: str, point: np.ndarray, frame: str):
    msg = PointStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame
    msg.point.x = point[0]
    msg.point.y = point[1]
    vis_pub_for_topic(node, topic).publish(msg)
