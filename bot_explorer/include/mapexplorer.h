#ifndef BOT_NAV_MAPEXPLORER_H
#define BOT_NAV_MAPEXPLORER_H
#pragma once

#include <cmath>
#include <chrono>
#include <memory>
#include <queue>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>

struct Frontier{ int x; int y;};

class MapExplorer : public rclcpp::Node{
public:
    MapExplorer();
    void map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
    bool is_valid_map(const nav_msgs::msg::OccupancyGrid & map);
    std::vector<std::vector<Frontier>> detect_frontiers(const nav_msgs::msg::OccupancyGrid & map);
    std::vector<Frontier> process_frontiers(const std::vector<std::vector<Frontier>> & clusters);
    Frontier publish_frontiers(const std::vector<Frontier> & frontiers, const std::vector<int> & cluster_n);
    void process_frontiers(const std::vector<Frontier> & frontiers);
private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_;
    nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;
    std::vector<std::vector<Frontier>> vec_frontier_clusters;
    std::vector<Frontier> vec_frontiers;
    bool map_received_{false};
};

#endif // BOT_NAV_MAPEXPLORER_H