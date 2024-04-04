# Utilities for F1/10

A collection of utilities for common tasks.

## Compilation

In order to use `utilities` in your ROS2 package, you must have the following. First, in your `package.xml`:
```xml
<depend>utilities</depend>
```

Second, and more importantly, in your `CMakeLists.txt`
```cmake
find_package(utilities REQUIRED)
ament_target_dependencies(target utilities)
```

## Modules

| Module     | Python Module                               | C++ Header                                             | Description                                               |
|------------|---------------------------------------------|--------------------------------------------------------|-----------------------------------------------------------|
| **params** | [`utilities.params`](./utilities/params.py) | [`"utilities/params.h"`](./include/utilities/params.h) | Easily declare and initialize ROS2 parameters             |
| **scan**   | [`utilities.scan`](./utilities/scan.py)     | [`"utilities/scan.h"`](./include/utilities/scan.h)     | Utilities for LIDAR range scans                           |
| **math**   | [`numpy`](https://numpy.org/doc/stable/)    | [`"utilities/math.h"`](./include/utilities/math.h)     | Common useful math functions                              |
| **vis**    | [`utilities.vis`](./utilities/vis.py)       | [`"utilities/vis.h"`](./include/utilities/vis.h)       | Visualize common ROS2 datatypes *without* the boilerplate |
| **se3**    | [`utilities.se3`](./utilities/se3.py)       | TODO                                                   | Operate on SE3 matrices generated from ROS2 messages      |

### Example Usage (Python)

```python
from utilities.params import vis_points
vis_points(...)
```

### Example Usage (C++)
C++ utility functions will typically be namespaced under their module name.

```cpp
#include "utilities/vis.h"
vis::vis_points(...)
```

## Tmux Sessions
[`tmux-sessions`](./tmux-sessions) contains session scripts for both hardware and simulation that generate a minimal testing workspace shown below.

```
+-----------------------------------------+     +-----------------------------------------+
|                    |                    |     |                                         |
|                    |                    |     |                                         |
|                    |         2          |     |                                         |
|                    |                    |     |                                         |
|                    |                    |     |                                         |
|         1          |--------------------|     |                    4                    |
|                    |                    |     |                                         |
|                    |                    |     |                                         |
|                    |         3          |     |                                         |
|                    |                    |     |                                         |
|                    |                    |     |                                         |
+-----------------------------------------+     +-----------------------------------------+

                  Window 1                                        Window 2
```

### Usage
While in your top-level workspace directory with a file-tree similar to the one provided:

```bash
<WORKSPACE> # <--- you are here
├── ...
└── src/
    ├── utilities # this repository
    └── <PACKAGE_NAME>
        ├── ...
        ├── src/
        │   └── <NODE_NAME>.cpp # if using a c++ node
        ├── scripts/
        │   └── <NODE_NAME> # if using a python node, NODE_NAME contains the .py extension
        └── config/
            └── <PARAMS_FILE>.yaml
```
The tmux session can be started with the following commands:

#### Simulation
```bash
$ ./src/utilities/tmux-sessions/sim-session.sh <PACKAGE_NAME> <NODE_NAME> <PARAMS_FILE>
```
The following commands are run in each pane:
| Pane | Command                                                                      |
|------|------------------------------------------------------------------------------|
| 1    | `ros2 launch f1tenth_gym_ros gym_bridge_launch.py`                           |
| 2    | `ros2 run <PACKAGE_NAME> <NODE_NAME> --ros-args --params-file <PARAMS_FILE>` |
| 3    | `vim <PARAMS_FILE>`                                                          |
| 4    | `colcon build --packages-up-to <PACKAGE_NAME>`


#### Hardware
```bash
$ ./src/utilities/tmux-sessions/hardware-session.sh <PACKAGE_NAME> <NODE_NAME> <PARAMS_FILE>
```

The following commands are run in each pane:
| Pane | Command                                                                      |
|------|------------------------------------------------------------------------------|
| 1    | `ros2 launch f1tenth_stack bringup_launch.py`                                |
| 2    | `ros2 run <PACKAGE_NAME> <NODE_NAME> --ros-args --params-file <PARAMS_FILE>` |
| 3    | `vim <PARAMS_FILE>`                                                          |
| 4    | `colcon build --packages-up-to <PACKAGE_NAME>`
