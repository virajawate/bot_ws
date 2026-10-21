
#ifndef BOT_NAV_MAPEXPLORER_H
#define BOT_NAV_MAPEXPLORER_H

#include <chrono>
#include <cstddef>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class MapExplorer : public rclcpp::Node
{
public:
    MapExplorer();

private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;

    using GoalHandleNav =
        rclcpp_action::ClientGoalHandle<NavigateToPose>;

    enum class State
    {
        WAITING_FOR_MAP,
        IDLE,
        NAVIGATING,
        RETRY_WAIT,
        COMPLETE
    };

    struct Frontier
    {
        double x{0.0};
        double y{0.0};
        double score{0.0};
        std::size_t size{1};
    };

    struct FailedFrontier
    {
        Frontier frontier;
        std::chrono::steady_clock::time_point failed_at;
    };

    // ROS callbacks
    void MapHandler(
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

    void ExplorationTick();

    // Map processing
    bool IsValidMap(
        const nav_msgs::msg::OccupancyGrid & map) const;

    std::vector<Frontier> DetectFrontiers(
        const nav_msgs::msg::OccupancyGrid & map) const;

    // Robot pose and task selection
    bool GetRobotPose(double & x, double & y);

    bool SelectFrontier(
        const std::vector<Frontier> & frontiers,
        double robot_x,
        double robot_y,
        Frontier & selected);

    void AssignNextGoal();

    // Nav2 action lifecycle
    void SendGoal(const Frontier & frontier);

    void HandleGoalResponse(
        GoalHandleNav::SharedPtr goal_handle);

    void HandleResult(
        const GoalHandleNav::WrappedResult & result);

    // Task management
    void RecordFailedFrontier(const Frontier & frontier);

    bool IsRecentlyFailed(const Frontier & frontier);

    void PruneFailedFrontiers();

    void PublishExplorationPoint(double x, double y);

    static double Distance(
        double x1, double y1,
        double x2, double y2);

    // ROS interfaces
    rclcpp::Subscription<
        nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PointStamped>::SharedPtr
        explore_point_pub_;

    rclcpp_action::Client<
        NavigateToPose>::SharedPtr goal_client_;

    rclcpp::TimerBase::SharedPtr exploration_timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::shared_ptr<tf2_ros::TransformListener>
        tf_listener_;

    // Latest map
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_;

    // Navigation state
    State state_{State::WAITING_FOR_MAP};

    GoalHandleNav::SharedPtr active_goal_;

    Frontier current_frontier_;

    bool map_received_{false};
    bool goal_request_pending_{false};

    // Failed-goal history
    std::vector<FailedFrontier> failed_frontiers_;

    // Configuration
    double free_threshold_{25.0};

    double min_frontier_separation_{0.50};

    double retry_distance_{0.50};

    double robot_radius_margin_{0.10};

    std::size_t min_frontier_size_{3};

    std::chrono::seconds retry_delay_{5};

    std::chrono::steady_clock::time_point next_retry_time_{};
};

#endif  // BOT_NAV_MAPEXPLORER_H
