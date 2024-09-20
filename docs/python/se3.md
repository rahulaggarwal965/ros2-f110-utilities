# SE(3)

The SE3 module provides a common interface to interact with the different transformation types within ROS through `numpy`. Additionally, we provide a few utility functions for using members of the SE(3) lie group within their matrix form.

## Example Usage

Imagine we wanted to take a LIDAR scan and transform it into the map frame using the localized pose from a robot's odometry. Here's how we could do that using this module.

```python
import numpy as np

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from utilities import se3

# Please see docs/python/scan.md for more information
from utilities.scan import scan_to_cloud

class Node(rclpy.Node):
    def __init__(self):
        super().__init__("cloud_transformer")

        self.scan_sub = self.create_subscription(LaserScan, self.config.scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, self.config.odom_topic, self.pose_callback, 10)

        self.map_T_base = np.eye(4)

    def pose_callback(self, odometry_msg: Odometry):
        # here, the `from_msg` function can get an SE(3) matrix from many types
        # of ROS messages
        self.map_T_base = se3.from_msg(odometry_msg.pose.pose)

    def scan_callback(self, scan_msg: LaserScan):
        # we use the map_T_base transformation we got from our pose callback
        # to transform the points in the laserscan to the map frame
        # We note here that we use the function `scan_to_cloud`
        self.cloud_map_frame = se3.transform_points(self.map_T_base, scan_to_cloud(scan_msg))
```