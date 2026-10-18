#include "mapexplorer.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <tf2/utils.h>

MapExplorer::MapExplorer() : Node("MapExplorer"){
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&MapExplorer::MapHandler, this, _1));
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
    const double yaw = tf2::getYaw(info.origin.orientation);

    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    for (std::size_t i = 0; i < map.size(); ++i)
    {
        const int occupancy = static_cast<int>(map[i]);

        if (occupancy == 0)
        {
            ++free_count;
            continue;
        }

        const bool occupied = occupancy > 0 && occupancy <= 100;
        const bool unknown = occupancy == -1;

        if (!occupied && !unknown)
            continue;

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
            ++unknown_count;
            unknown_x_sum += x;
            unknown_y_sum += y;
        }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Cells: free=%zu, occupied=%zu, unknown=%zu",
        free_count, occupied_count, unknown_count);

    if (occupied_count > 0)
    {
        const double occupied_x =
            occupied_x_sum / occupied_count;
        const double occupied_y =
            occupied_y_sum / occupied_count;

        RCLCPP_INFO(
            this->get_logger(),
            "Occupied centroid: (%.3f, %.3f) m",
            occupied_x, occupied_y);
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
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "No unknown cells");
    }
}