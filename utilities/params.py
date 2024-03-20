from rclpy.node import Node
from typing import TypeVar
T = TypeVar("T")

__all__ = ["get_param"]

_TYPE_TO_VALUE = {
    float: "double_value",
    str: "string_value",
    int: "integer_value",
    bool: "bool_value",
}

_ELEMENT_TO_VALUE = {
    float: "double_array_value",
    int: "integer_array_value"
}

def get_param(
    node: Node,
    name: str,
    default: T
) -> T:
    """Declares a named parameter and returns its typed value

    Parameters
    ----------
    node : Node
        The ROS2 node on which to declare the parameter
    name : str
        The name of the parameter
    default : T
        The default value for the parameter

    Returns
    -------
    T
        The value of the parameter (typed)
    """
    param_type = type(default)
    if param_type == list:
        param_type_value = _ELEMENT_TO_VALUE[type(default[0])]
    else:
        param_type_value = _TYPE_TO_VALUE[param_type]

    node.declare_parameter(name, default)
    param = node.get_parameter(name).get_parameter_value().__getattribute__(param_type_value)
    if param_type == list:
        param = param.tolist()
    node.get_logger().info(f"{name.upper()}: {param}")
    return param

