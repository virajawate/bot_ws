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
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

struct Frontier{ int x; int y;};

class MapExplorer : public rclcpp::Node{
public:
    MapExplorer();
    void map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
    bool is_valid_map(const nav_msgs::msg::OccupancyGrid & map);
    void pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    std::vector<std::vector<Frontier>> detect_frontiers(const nav_msgs::msg::OccupancyGrid & map);
    Frontier process_frontiers(const std::vector<std::vector<Frontier>> & clusters);
    Frontier publish_frontiers(const std::vector<Frontier> & frontiers, const std::vector<int> & cluster_n);
    bool navigate_to_frontier(const Frontier & frontier);
private:
    void send_navigation_goal();
    void handle_navigation_failure(const char * reason);

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navigate_action_client_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_;
    nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;
    geometry_msgs::msg::PoseWithCovarianceStamped curr_pose_;
    nav2_msgs::action::NavigateThroughPoses::Goal navigation_goal_;
    std::vector<std::vector<Frontier>> vec_frontier_clusters;
    bool map_received_{false}, goal_reached{false}, navigation_active_{false};
    bool failed_frontier_valid_{false}, exploration_stopped_{false};
    unsigned int navigation_attempt_{0};
    Frontier frontier2nav, navigation_frontier_, failed_frontier_;
};

#endif // BOT_NAV_MAPEXPLORER_H