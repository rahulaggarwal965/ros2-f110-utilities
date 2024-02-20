#pragma once

#include <cctype>
#include <rclcpp/rclcpp.hpp>

template<typename T>
T get_param(rclcpp::Node &node, const std::string &param_name, T default_value) {
    node.declare_parameter(param_name, default_value);
    const auto param = node.get_parameter(param_name).get_value<T>();
    auto upper_param_name = param_name;
    std::transform(upper_param_name.begin(), upper_param_name.end(), upper_param_name.begin(), ::toupper);
    RCLCPP_INFO_STREAM(node.get_logger(), upper_param_name << ": " << param);
    return param;
}
