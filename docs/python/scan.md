# Scan

The scan module provides a few key functions to make interacting with LIDAR scans in a ROS msg format easier. Specifically, we offer conversion from messages to `numpy` arrays as well as some utilities to query certain indices and angles.

## Example Usage

Imagine we wanted to get the range data from a LIDAR scan for a *specific angle*. Here is how we would do that using the `scan` module.

```python
import numpy as np

import rclpy
from sensor_msgs.msg import LaserScan

from utilities.scan import scan_to_cloud, index_at_angle

class Node(rclpy.Node):
    def __init__(self):
        super().__init__("scan_info")

        self.scan_sub = self.create_subscription(LaserScan, self.config.scan_topic, self.scan_callback, 10)

    def scan_callback(self, scan_msg: LaserScan):
        self.cloud = scan_to_cloud(scan_msg)

        left_range_index = index_at_angle(-90.0, scan_msg.angle_min, scan_msg.angle_increment)
        right_range_index = index_at_angle(90.0, scan_msg.angle_min, scan_msg.angle_increment)

        left_range, right_range = self.cloud[[left_range_index, right_range_index]]
```