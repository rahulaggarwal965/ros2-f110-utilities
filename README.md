# Utilities for F1/10

A collection of utilities for common tasks.

## Usage

In order to use `utilities` in your ROS2 package, you must have the following. First, in your `package.xml`:
```xml
<depend>utilities</depend>
```

Second, and more importantly, in your `CMakeLists.txt`
```cmake
find_package(utilities REQUIRED)
ament_target_dependencies(target utilities)
```

## Python

### [Params](./utilities/params.py)
This module is designed to make declaring and getting parameter values exceptionally easy.

```python
from utilities.params import ...
```

### [Scan](./utilities/scan.py)
This module is designed to make dealing with LaserScan messages easier

```python
from utilities.scan import ...
```

### [Visualization](./utilities/vis.py)
This module is designed to make debugging through visualization easier. You can visualize things like ranges and points without having to write the cruft.

```python
from utilities.vis import ...
```

## C++ (Partial)

### [Params](./include/utilities/params.h)
This module is designed to make declaring and getting parameter values exceptionally easy.

```cpp
#include "utilities/params.h"
```
