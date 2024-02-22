#pragma once

#include <cctype>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <unordered_map>


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

namespace vis {

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
    _pub_to_topic(node, topic, msg);
}

inline void vis_point(
    rclcpp::Node &node,
    const std::string &topic,
    const std::array<float, 2> &point,
    const std::string &frame
) {
    using PointStamped = geometry_msgs::msg::PointStamped;
    PointStamped msg;
    msg.header.stamp = node.get_clock()->now();
    msg.header.frame_id = frame;
    msg.point.x = point[0];
    msg.point.y = point[1];
    _pub_to_topic(node, topic, msg);
}

}
