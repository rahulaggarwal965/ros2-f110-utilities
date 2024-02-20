from rclpy.node import Node

__all__ = ["get_param"]

def get_param(node: Node, name: str, default):
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

