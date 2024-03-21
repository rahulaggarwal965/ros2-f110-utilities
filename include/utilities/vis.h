#pragma once

#include <cctype>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <unordered_map>

// TODO(rahul): transition to Eigen
namespace vis {

namespace internal {

template<typename T>
static void _pub_to_topic(rclcpp::Node &node, const std::string &topic,  const T &msg) {
    static std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr> topic_to_pub;
    if (topic_to_pub.find(topic) == topic_to_pub.end()) {
        topic_to_pub[topic] = node.create_publisher<T>(topic, 10);
    }
    if (auto publisher = dynamic_cast<rclcpp::Publisher<T>*>(topic_to_pub[topic].get())) {
        publisher->publish(msg);
    }
}

static void _vis_points(
    rclcpp::Node &node,
    const std::string &topic,
    const std::vector<geometry_msgs::msg::Point> &points,
    int marker_type,
    const std::string &frame,
    float scale = 0.1f,
    const std::array<float, 4> &color = {1.0f, 0.0f, 0.0f, 1.0f}
) {
    using Marker = visualization_msgs::msg::Marker;
    Marker msg;
    msg.header.stamp = node.get_clock()->now();
    msg.header.frame_id = frame;
    msg.ns = topic;
    msg.id = 0;
    msg.type = marker_type;
    msg.action = Marker::ADD;

    msg.points = points;

    msg.scale.x = scale;
    msg.scale.y = scale;
    msg.scale.z = scale;

    msg.color.r = color[0];
    msg.color.g = color[1];
    msg.color.b = color[2];
    msg.color.a = color[3];

    _pub_to_topic(node, topic, msg);
}

}

inline void vis_scan(
    rclcpp::Node &node,
    const std::string &topic,
    const std::vector<float> &ranges,
    const std::string &frame,
    float angle_min,
    float angle_increment,
    float range_min = 0.0,
    float range_max = 30.0
) {
    using LaserScan = sensor_msgs::msg::LaserScan;
    LaserScan msg; 
    msg.header.stamp = node.get_clock()->now();
    msg.header.frame_id = frame;
    msg.ranges = ranges;
    msg.angle_min = angle_min;
    msg.angle_increment = angle_increment;
    msg.range_min = range_min;
    msg.range_max = range_max;
    msg.angle_max = angle_min + ranges.size() * angle_increment;
    internal::_pub_to_topic(node, topic, msg);
}

inline void vis_points(
    rclcpp::Node &node,
    const std::string &topic,
    const std::vector<geometry_msgs::msg::Point> &points,
    const std::string &frame,
    float scale = 0.1f,
    const std::array<float, 4> &color = {1.0f, 0.0f, 0.0f, 1.0f}
) {
    using Marker = visualization_msgs::msg::Marker;
    internal::_vis_points(node, topic, points, Marker::POINTS, frame, scale, color);
}

inline void vis_point(
    rclcpp::Node &node,
    const std::string &topic,
    const geometry_msgs::msg::Point &point,
    const std::string &frame,
    float scale = 0.1f,
    const std::array<float, 4> &color = {1.0f, 0.0f, 0.0f, 1.0f}
) { vis_points(node, topic, {point}, frame, scale, color); }

inline void vis_point(
    rclcpp::Node &node,
    const std::string &topic,
    const std::array<float, 2> &point,
    const std::string &frame,
    float scale = 0.1f,
    const std::array<float, 4> &color = {1.0f, 0.0f, 0.0f, 1.0f}
) {
    using Point = geometry_msgs::msg::Point;
    Point p;
    p.x = point[0];
    p.y = point[1];

    vis_point(node, topic, p, frame, scale, color);
}

inline void vis_path(
    rclcpp::Node &node,
    const std::string &topic,
    const std::vector<geometry_msgs::msg::Point> &points,
    const std::string &frame,
    float scale = 0.1f,
    const std::array<float, 4> &color = {1.0f, 0.0f, 0.0f, 1.0f}
) {
    using Marker = visualization_msgs::msg::Marker;
    internal::_vis_points(node, topic, points, Marker::LINE_STRIP, frame, scale, color);
}

// The `lines` parameter should contain an array of points such that every unique pair
// (0,1), (2,3), ... will form a line. This differs slightly from the python API.
inline void vis_lines(
    rclcpp::Node &node,
    const std::string &topic,
    const std::vector<geometry_msgs::msg::Point> &lines,
    const std::string &frame,
    float scale = 0.1f,
    const std::array<float, 4> &color = {1.0f, 0.0f, 0.0f, 1.0f}
) {
    using Marker = visualization_msgs::msg::Marker;
    internal::_vis_points(node, topic, lines, Marker::LINE_LIST, frame, scale, color);
}

}
