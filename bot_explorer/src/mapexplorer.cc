#include "mapexplorer.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <tf2/utils.hpp>

MapExplorer::MapExplorer() : Node("MapExplorer"){
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&MapExplorer::MapHandler, this, _1));
    explore_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/explorer/points", 10);
    goal_client = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
}

void MapExplorer::ExplorerPointPub(double x, double y){
    geometry_msgs::msg::PointStamped exp_pnt;

    exp_pnt.header.stamp = this->now();
    exp_pnt.header.frame_id = "map";
    exp_pnt.point.x = x;
    exp_pnt.point.y = y;
    exp_pnt.point.z = 0;
    explore_point_pub_->publish(exp_pnt);
    SendGoal(x, y);
    RCLCPP_INFO(this->get_logger(), "Publishing point : (%.3f, %.3f)", x, y);

}

void MapExplorer::SendGoal(double x, double y){
    if(!goal_client->action_server_is_ready()){
        RCLCPP_WARN(this->get_logger(), "Nav2Goal is not ready yet.");
        return;
    }
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = this->now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.position.z = 0;
    goal.pose.pose.orientation.x = 0;
    goal.pose.pose.orientation.y = 0;
    goal.pose.pose.orientation.z = 0;
    goal.pose.pose.orientation.w = 1;

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback = [this](GoalHandlerNav::SharedPtr goal_handle){
        if(!goal_handle){
            RCLCPP_ERROR(this->get_logger(), "Nav2 rejected the goal.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Nav2 accepted the goal.");
        }
    };
    options.result_callback = [this](const GoalHandlerNav::WrappedResult &result){
        switch(result.code){
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Navigation Succeeded.");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO(this->get_logger(), "Navigator Aborted.");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(),"Navigator Cancelled.");
                break;
            case rclcpp_action::ResultCode::UNKNOWN:
                RCLCPP_INFO(this->get_logger(),"Navigator Unknown.");
                break;
        }
    };
    goal_client->async_send_goal(goal, options);
}

void MapExplorer::MapHandler(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_msg)
{
    const auto & map = map_msg->data;
    const auto & info = map_msg->info;

    if (info.width == 0 || info.height == 0 ||
        info.resolution <= 0.0 ||
        map.size() != static_cast<std::size_t>(info.width) * info.height)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid occupancy grid");
        return;
    }

    std::size_t free_count = 0;
    std::size_t occupied_count = 0;
    std::size_t unknown_count = 0;

    double occupied_x_sum = 0.0;
    double occupied_y_sum = 0.0;

    double unknown_x_sum = 0.0;
    double unknown_y_sum = 0.0;

    const double resolution = info.resolution;
    const double origin_x = info.origin.position.x;
    const double origin_y = info.origin.position.y;
    const auto & q = info.origin.orientation;

    const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    );

    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    for (std::size_t i = 0; i < map.size(); ++i){
        const int occupancy = static_cast<int>(map[i]);

        if (occupancy == 0){
            ++free_count;
            continue;
        }

        const bool occupied = occupancy > 0 && occupancy <= 100;
        const bool unknown = occupancy == -1;

        if (!occupied && !unknown) continue;

        // Convert linear index to grid coordinates.
        const std::size_t cell_x = i % info.width;
        const std::size_t cell_y = i / info.width;

        // Calculate cell center in the grid's local coordinates.
        const double local_x = (cell_x + 0.5) * resolution;
        const double local_y = (cell_y + 0.5) * resolution;

        // Transform cell center into the map frame.
        const double x =
            origin_x + local_x * cos_yaw - local_y * sin_yaw;

        const double y =
            origin_y + local_x * sin_yaw + local_y * cos_yaw;

        if (occupied)
        {
            ++occupied_count;
            occupied_x_sum += x;
            occupied_y_sum += y;
        }
        else
        {
            // ExplorerPointPub(x, y);
            ++unknown_count;
            unknown_x_sum += x;
            unknown_y_sum += y;
        }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Cells: free=%zu, occupied=%zu, unknown=%zu",
        free_count, occupied_count, unknown_count
    );

    if (occupied_count > 0){
        const double occupied_x =
            occupied_x_sum / occupied_count;
        const double occupied_y =
            occupied_y_sum / occupied_count;

        RCLCPP_INFO(
            this->get_logger(),
            "Occupied centroid: (%.3f, %.3f) m",
            occupied_x, occupied_y
        );
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "No occupied cells");
    }

    if (unknown_count > 0)
    {
        const double unknown_x =
            unknown_x_sum / unknown_count;
        const double unknown_y =
            unknown_y_sum / unknown_count;
        
        RCLCPP_INFO(
            this->get_logger(),
            "Unknown centroid: (%.3f, %.3f) m",
            unknown_x, unknown_y);
        ExplorerPointPub(unknown_x, unknown_y);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "No unknown cells");
    }
}