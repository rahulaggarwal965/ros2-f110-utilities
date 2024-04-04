import dataclasses
from typing import List, TypeVar

from deprecated import deprecated
from rclpy.node import Node, SetParametersResult
from rclpy.parameter import Parameter, ParameterType, ParameterValue

T = TypeVar("T")

__all__ = ["get_param", "register_config"]

# NOTE(rahul): can be removed if ROS2 is Humble or greater: https://github.com/ros2/rclpy/blob/32e4eae63cc3450309e7bf1f9dacf3b725a18d88/rclpy/rclpy/parameter.py#L240
def _parameter_value_to_python(parameter_value: ParameterValue):
    """
    Get the value for the Python builtin type from a rcl_interfaces/msg/ParameterValue object.

    Returns the value member of the message based on the ``type`` member.
    Returns ``None`` if the parameter is "NOT_SET".

    :param parameter_value: The message to get the value from.
    :raises RuntimeError: if the member ``type`` has an unexpected value.
    """
    if parameter_value.type == ParameterType.PARAMETER_BOOL:
        value = parameter_value.bool_value
    elif parameter_value.type == ParameterType.PARAMETER_INTEGER:
        value = parameter_value.integer_value
    elif parameter_value.type == ParameterType.PARAMETER_DOUBLE:
        value = parameter_value.double_value
    elif parameter_value.type == ParameterType.PARAMETER_STRING:
        value = parameter_value.string_value
    elif parameter_value.type == ParameterType.PARAMETER_BYTE_ARRAY:
        value = list(parameter_value.byte_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_BOOL_ARRAY:
        value = list(parameter_value.bool_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
        value = list(parameter_value.integer_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        value = list(parameter_value.double_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_STRING_ARRAY:
        value = list(parameter_value.string_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_NOT_SET:
        value = None
    else:
        raise RuntimeError(f'unexpected parameter type {parameter_value.type}')

    return value

@deprecated(reason = "get_param does not implement parameter updates. Use a dataclass config and the register_config function instead.")
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
    node.declare_parameter(name, default)
    param = _parameter_value_to_python(node.get_parameter(name).get_parameter_value())
    node.get_logger().info(f"{name.upper()}: {param}")
    return param

def register_config(node: Node, config: T) -> T:
    """Registers a dataclass config representing a set of ROS2 parameters. Provides the
    infrastructure for parameter updates.

    Parameters
    ----------
    node : Node
        The ROS2 node on which to declare the parameters
    config : T
        A dataclass corresponding to a set of default ROS2 parameters

    Returns
    -------
    T
        A dataclass corresponding to the newly-set ROS2 parameters.
    """
    assert dataclasses.is_dataclass(config) and not isinstance(config, type)
    config_dict = dataclasses.asdict(config)

    # TODO(rahul): handle nested dataclassses/dicts
    for param_name, param_default_value in config_dict.items():
        node.declare_parameter(param_name, param_default_value)
        param_value = _parameter_value_to_python(node.get_parameter(param_name).get_parameter_value())
        config.__setattr__(param_name, param_value)
        node.get_logger().info(f"{param_name.upper()}: {param_value}")

    def parameters_callback(params: List[Parameter]) -> SetParametersResult:
        for param in params:
            new_param_value = _parameter_value_to_python(param.get_parameter_value())
            config.__setattr__(param.name, new_param_value)
            node.get_logger().info(f"{param.name.upper()}: {new_param_value}")

        return SetParametersResult(successful=True)

    node.add_on_set_parameters_callback(parameters_callback)

    return config
