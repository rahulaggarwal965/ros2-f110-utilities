# Params

The params module provides an easy, intuitive way to define a set of parameters for your ROS nodes using python native dataclasses. Additionally, it handles all logging and updating of parameters in the background.

## Example Usage

The configuration is defined entirely in code.

```python
import rclpy
from sensor_msgs.msg import LaserScan

from dataclasses import dataclass, field
from utilities.params import register_config

@dataclass
class NodeConfig:
    # fields can be basic types
    scan_topic: str = "/scan"
    map_frame: str = "map"
    map_file: str = "./src/pure_pursuit/maps/levine_scan_1.pgm"

    speed_factor: int = 7

    # but they can also be more complex types
    grid_dimension: List[float] = field(default_factory=lambda: [5.0, 10.0])

    # or bools
    visualize: bool = True

    # they can even be other dataclass configs
    steer_controller: PDControllerConfig = PDControllerConfig(kp=0.5, kd=0)

class Node(rclpy.Node):
    def __init__(self):
        super().__init__("node")

        # register config using this syntax
        self.config = register_config(self, NodeConfig())

        # usage of a config parameter
        self.scan_sub = self.create_subscription(LaserScan, self.config.scan_topic, self.scan_callback, 10)

        ...
```

From there, we can write a yaml file that we can specify at runtime to fill in our parameters.

```yaml
node:
    ros__parameters:

        scan_topic: /scan

        map_frame: map

        map_file: ./src/pure_pursuit/maps/levine_scan_1.pgm

        # we change the value from the default here
        speed_factor: 3

        grid_dimension: [2, 2]

        visualize: false

        steer_controller:
            kp: 0.7
            kd: 0.0
```

## How it Works

When we register a config on a ROS node, we do a couple of things.

1. We recursively serialize the config structure providing an absolute name for each configuration variable.

2. We declare these absolute parameter names on the node class, which allows the ros param server to find them in the ros param tree and initialize them for our node.

3. We register a callback that listens to when any parameter changes and updates our configuration to reflect said changes.
