from rclpy.node import Node
from typing import Union

__all__ = ["get_param"]

def get_param(
    node: Node,
    name: str,
    default: Union[float, str, int, bool]
) -> Union[float, str, int, bool]:
    """Declares a named parameter and returns its typed value

    Parameters
    ----------
    node : Node
        The ROS2 node on which to declare the parameter
    name : str
        The name of the parameter
    default : Union[float, str, int, bool]
        The default value for the parameter (must be typed)

    Returns
    -------
    Union[float, str, int, bool]
        The value of the parameter (typed)
    """
    type_to_value = {
        float: "double_value",
        str: "string_value",
        int: "integer_value",
        bool: "bool_value"
    }
    node.declare_parameter(name, default)
    param = node.get_parameter(name).get_parameter_value().__getattribute__(type_to_value[type(default)])
    node.get_logger().info(f"{name.upper()}: {param}")
    return param

