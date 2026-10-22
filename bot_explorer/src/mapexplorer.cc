
#include "mapexplorer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

#include "tf2/exceptions.hpp"
#include "tf2/time.h"

using namespace std::chrono_literals;

MapExplorer::MapExplorer()
    : Node("map_explorer")
{
    // Subscribe to the live occupancy grid.
    map_sub_ = this->create_subscription<
        nav_msgs::msg::OccupancyGrid>(
        "/map",
        rclcpp::QoS(1),
        std::bind(
            &MapExplorer::MapHandler,
            this,
            std::placeholders::_1));

    // Publish the selected exploration target.
    explore_point_pub_ =
        this->create_publisher<
            geometry_msgs::msg::PointStamped>(
            "/explorer/points", 10);

    // Nav2 GoalPose.
    goal_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10);

    // TF: map -> base_footprint.
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
        this->get_clock());

    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(
            *tf_buffer_);

    // Task assignment runs independently of map reception.
    exploration_timer_ = this->create_wall_timer(
        1s,
        std::bind(
            &MapExplorer::ExplorationTick,
            this));
    unknown_marker_pub_ =
        this->create_publisher<
            visualization_msgs::msg::MarkerArray>(
            "/explorer/unknown_markers", 10);
    RCLCPP_INFO(
        this->get_logger(),
        "MapExplorer initialized.");
}

// ------------------------------------------------------------
// Map validation and reception
// ------------------------------------------------------------

bool MapExplorer::IsValidMap(
    const nav_msgs::msg::OccupancyGrid & map) const
{
    const auto & info = map.info;

    if (info.width == 0 ||
        info.height == 0 ||
        !std::isfinite(info.resolution) ||
        info.resolution <= 0.0)
    {
        return false;
    }

    const std::size_t expected =
        static_cast<std::size_t>(info.width) *
        static_cast<std::size_t>(info.height);

    if (map.data.size() != expected)
        return false;

    const auto & q = info.origin.orientation;

    if (!std::isfinite(info.origin.position.x) ||
        !std::isfinite(info.origin.position.y) ||
        !std::isfinite(q.x) ||
        !std::isfinite(q.y) ||
        !std::isfinite(q.z) ||
        !std::isfinite(q.w))
    {
        return false;
    }

    const double q_norm =
        q.x * q.x + q.y * q.y +
        q.z * q.z + q.w * q.w;

    return std::isfinite(q_norm) && q_norm > 1e-12;
}

void MapExplorer::MapHandler(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
    if (!msg || !IsValidMap(*msg))
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            3000,
            "Ignoring invalid occupancy grid.");
        return;
    }

    latest_map_ = std::move(msg);
    map_received_ = true;

    PublishUnknownVisualization(*latest_map_);
    if (state_ == State::WAITING_FOR_MAP ||
        state_ == State::COMPLETE)
    {
        state_ = State::IDLE;
    }
}

// ------------------------------------------------------------
// Frontier detection
// ------------------------------------------------------------

std::vector<MapExplorer::Frontier>
MapExplorer::DetectFrontiers(
    const nav_msgs::msg::OccupancyGrid & map) const
{
    std::vector<Frontier> frontiers;

    const auto & info = map.info;

    const std::size_t width = info.width;
    const std::size_t height = info.height;

    const double resolution = info.resolution;

    const double origin_x = info.origin.position.x;
    const double origin_y = info.origin.position.y;

    const auto & q = info.origin.orientation;

    // Extract the origin orientation's yaw.
    const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    const auto index =
        [width](std::size_t x, std::size_t y)
        {
            return y * width + x;
        };

    const auto is_free =
        [&](std::size_t x, std::size_t y)
        {
            const int value =
                static_cast<int>(map.data[index(x, y)]);

            return value >= 0 &&
                   value <= free_threshold_;
        };

    const auto is_unknown =
        [&](std::size_t x, std::size_t y)
        {
            return map.data[index(x, y)] == -1;
        };

    // 4-connected neighborhood.
    constexpr std::array<int, 4> dx = {-1, 1, 0, 0};
    constexpr std::array<int, 4> dy = {0, 0, -1, 1};

    for (std::size_t y = 0; y < height; ++y)
    {
        for (std::size_t x = 0; x < width; ++x)
        {
            // A frontier target must be known free space.
            if (!is_free(x, y))
                continue;

            bool is_frontier = false;

            for (std::size_t k = 0; k < dx.size(); ++k)
            {
                const int nx =
                    static_cast<int>(x) + dx[k];

                const int ny =
                    static_cast<int>(y) + dy[k];

                // Explicitly handle map boundaries.
                if (nx < 0 || ny < 0 ||
                    nx >= static_cast<int>(width) ||
                    ny >= static_cast<int>(height))
                {
                    continue;
                }

                if (is_unknown(
                        static_cast<std::size_t>(nx),
                        static_cast<std::size_t>(ny)))
                {
                    is_frontier = true;
                    break;
                }
            }

            if (!is_frontier)
                continue;

            // Cell center in grid-local coordinates.
            const double local_x =
                (static_cast<double>(x) + 0.5) * resolution;

            const double local_y =
                (static_cast<double>(y) + 0.5) * resolution;

            Frontier frontier;

            // Transform into the map frame.
            frontier.x =
                origin_x +
                local_x * cos_yaw -
                local_y * sin_yaw;

            frontier.y =
                origin_y +
                local_x * sin_yaw +
                local_y * cos_yaw;

            frontiers.push_back(frontier);
        }
    }

    return frontiers;
}

// ------------------------------------------------------------
// Robot pose
// ------------------------------------------------------------

bool MapExplorer::GetRobotPose(
    double & x, double & y)
{
    try
    {
        const auto transform =
            tf_buffer_->lookupTransform(
                "map",
                "base_footprint",
                tf2::TimePointZero);

        x = transform.transform.translation.x;
        y = transform.transform.translation.y;

        return std::isfinite(x) && std::isfinite(y);
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            3000,
            "Robot pose unavailable: %s",
            ex.what());

        return false;
    }
}

// ------------------------------------------------------------
// Task selection
// ------------------------------------------------------------

double MapExplorer::Distance(
    double x1, double y1,
    double x2, double y2)
{
    return std::hypot(x2 - x1, y2 - y1);
}

void MapExplorer::PruneFailedFrontiers()
{
    const auto now =
        std::chrono::steady_clock::now();

    // Expire failed targets after one minute.
    constexpr auto failure_memory = 60s;

    failed_frontiers_.erase(
        std::remove_if(
            failed_frontiers_.begin(),
            failed_frontiers_.end(),
            [&](const FailedFrontier & failed)
            {
                return now - failed.failed_at >
                       failure_memory;
            }),
        failed_frontiers_.end());
}

bool MapExplorer::IsRecentlyFailed(
    const Frontier & frontier)
{
    PruneFailedFrontiers();

    for (const auto & failed : failed_frontiers_)
    {
        if (Distance(
                frontier.x, frontier.y,
                failed.frontier.x,
                failed.frontier.y) < retry_distance_)
        {
            return true;
        }
    }

    return false;
}

void MapExplorer::RecordFailedFrontier(
    const Frontier & frontier)
{
    failed_frontiers_.push_back(
        FailedFrontier{
            frontier,
            std::chrono::steady_clock::now()});

    PruneFailedFrontiers();
}

bool MapExplorer::SelectFrontier(
    const std::vector<Frontier> & frontiers,
    double robot_x,
    double robot_y,
    Frontier & selected)
{
    bool found = false;

    double best_score =
        std::numeric_limits<double>::infinity();

    for (const auto & frontier : frontiers)
    {
        const double distance =
            Distance(
                robot_x, robot_y,
                frontier.x, frontier.y);

        // Reject targets too close to the robot.
        if (distance < min_frontier_separation_)
            continue;

        // Do not immediately retry a failed target.
        if (IsRecentlyFailed(frontier))
            continue;

        // Baseline scoring: nearest frontier first.
        // Path cost and information gain can be added later.
        const double score = distance;

        if (score < best_score)
        {
            best_score = score;
            selected = frontier;
            selected.score = score;
            found = true;
        }
    }

    return found;
}

// ------------------------------------------------------------
// Exploration state machine
// ------------------------------------------------------------
void MapExplorer::ExplorationTick(){
    if (!map_received_ || !latest_map_)
        return;

    if (state_ == State::NAVIGATING ||
        state_ == State::COMPLETE)
        return;

    if (state_ == State::RETRY_WAIT)
    {
        if (std::chrono::steady_clock::now() <
            next_retry_time_)
            return;

        state_ = State::IDLE;
    }

    AssignNextGoal();
}

void MapExplorer::AssignNextGoal()
{
    if (!latest_map_ ||
        !IsValidMap(*latest_map_))
    {
        return;
    }

    double robot_x = 0.0;
    double robot_y = 0.0;

    if (!GetRobotPose(robot_x, robot_y))
        return;

    const auto frontiers =
        DetectFrontiers(*latest_map_);

    RCLCPP_DEBUG(
        get_logger(),
        "Detected %zu frontier cells.",
        frontiers.size());

    Frontier selected;

    if (!SelectFrontier(
            frontiers,
            robot_x,
            robot_y,
            selected))
    {
        state_ = State::COMPLETE;

        RCLCPP_INFO(
            get_logger(),
            "No eligible frontier. Waiting for a map update.");

        return;
    }

    RCLCPP_INFO(
        get_logger(),
        "Selected frontier: x=%.2f, y=%.2f, "
        "distance=%.2f m",
        selected.x,
        selected.y,
        selected.score);

    SendGoal(selected);
}

// ------------------------------------------------------------
// Nav2 goal submission
// ------------------------------------------------------------
void MapExplorer::SendGoal(const Frontier & frontier){
    geometry_msgs::msg::PoseStamped goal;

    goal.header.frame_id = "map";
    goal.header.stamp = this->now();

    goal.pose.position.x = frontier.x;
    goal.pose.position.y = frontier.y;
    goal.pose.position.z = 0.0;

    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;

    current_frontier_ = frontier;

    PublishExplorationPoint(frontier.x, frontier.y);
    goal_pose_pub_->publish(goal);

    RCLCPP_INFO(
        get_logger(),
        "Published exploration goal to /goal_pose: "
        "x=%.2f, y=%.2f",
        frontier.x,
        frontier.y);

    // Topic publishing provides no navigation result callback.
    // Avoid repeatedly publishing the same goal every timer tick.
    state_ = State::NAVIGATING;
}

void MapExplorer::HandleGoalResponse(
    GoalHandleNav::SharedPtr goal_handle)
{
    goal_request_pending_ = false;

    if (!goal_handle)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Nav2 rejected the exploration goal.");

        RecordFailedFrontier(current_frontier_);

        next_retry_time_ =
            std::chrono::steady_clock::now() +
            retry_delay_;

        state_ = State::RETRY_WAIT;

        return;
    }

    active_goal_ = goal_handle;

    RCLCPP_INFO(
        get_logger(),
        "Nav2 accepted the exploration goal.");
}

void MapExplorer::HandleResult(
    const GoalHandleNav::WrappedResult & result)
{
    active_goal_.reset();
    goal_request_pending_ = false;

    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            RCLCPP_INFO(
                get_logger(),
                "Exploration goal succeeded.");

            state_ = State::IDLE;
            break;
        }

        case rclcpp_action::ResultCode::ABORTED:
        {
            RCLCPP_WARN(
                get_logger(),
                "Navigation aborted. Retiring this target temporarily.");

            RecordFailedFrontier(current_frontier_);

            next_retry_time_ =
                std::chrono::steady_clock::now() +
                retry_delay_;

            state_ = State::RETRY_WAIT;
            break;
        }

        case rclcpp_action::ResultCode::CANCELED:
        {
            RCLCPP_INFO(
                get_logger(),
                "Navigation canceled.");

            state_ = State::IDLE;
            break;
        }

        default:
        {
            RCLCPP_WARN(
                get_logger(),
                "Navigation returned an unknown result.");

            RecordFailedFrontier(current_frontier_);

            next_retry_time_ =
                std::chrono::steady_clock::now() +
                retry_delay_;

            state_ = State::RETRY_WAIT;
            break;
        }
    }
}

// ------------------------------------------------------------
// Exploration target publisher
// ------------------------------------------------------------

void MapExplorer::PublishExplorationPoint(
    double x, double y)
{
    geometry_msgs::msg::PointStamped point;

    point.header.stamp = this->now();
    point.header.frame_id = "map";

    point.point.x = x;
    point.point.y = y;
    point.point.z = 0.0;

    explore_point_pub_->publish(point);
}

void MapExplorer::PublishUnknownVisualization(
    const nav_msgs::msg::OccupancyGrid & map)
{
    visualization_msgs::msg::MarkerArray markers;

    const auto & info = map.info;

    const int width = static_cast<int>(info.width);
    const int height = static_cast<int>(info.height);
    const double resolution = info.resolution;

    const auto & origin = info.origin.position;
    const auto & q = info.origin.orientation;

    const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    auto toMap = [&](double lx, double ly)
    {
        geometry_msgs::msg::Point p;
        p.x = origin.x + lx * cos_yaw - ly * sin_yaw;
        p.y = origin.y + lx * sin_yaw + ly * cos_yaw;
        p.z = 0.05;
        return p;
    };

    visualization_msgs::msg::Marker unknown;
    unknown.header.frame_id = map.header.frame_id;
    unknown.header.stamp = this->now();
    unknown.ns = "unknown_cells";
    unknown.id = 0;
    unknown.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    unknown.action = visualization_msgs::msg::Marker::ADD;

    unknown.pose.orientation.w = 1.0;
    unknown.scale.x = resolution * 0.25;
    unknown.scale.y = resolution * 0.25;
    unknown.scale.z = resolution * 0.25;

    unknown.color.r = 1.0;
    unknown.color.g = 0.1;
    unknown.color.b = 0.1;
    unknown.color.a = 0.9;

    visualization_msgs::msg::Marker sides;
    sides.header = unknown.header;
    sides.ns = "unknown_side_centers";
    sides.id = 1;
    sides.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    sides.action = visualization_msgs::msg::Marker::ADD;

    sides.pose.orientation.w = 1.0;
    sides.scale.x = resolution * 0.18;
    sides.scale.y = resolution * 0.18;
    sides.scale.z = resolution * 0.18;

    sides.color.r = 0.1;
    sides.color.g = 1.0;
    sides.color.b = 0.1;
    sides.color.a = 1.0;

    constexpr int dx[] = {-1, 1, 0, 0};
    constexpr int dy[] = {0, 0, -1, 1};

    const auto index = [width](int x, int y)
    {
        return static_cast<std::size_t>(y) *
                   static_cast<std::size_t>(width) +
               static_cast<std::size_t>(x);
    };

    auto isUnknown = [&](int x, int y)
    {
        return map.data[index(x, y)] == -1;
    };

    auto isFree = [&](int x, int y)
    {
        const int value = map.data[index(x, y)];
        return value >= 0 && value <= free_threshold_;
    };

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            if (isUnknown(x, y))
            {
                // Center of every unknown occupancy-grid cell.
                unknown.points.push_back(
                    toMap(
                        (x + 0.5) * resolution,
                        (y + 0.5) * resolution));
            }

            if (!isFree(x, y))
                continue;

            // Find each side of this free cell that borders
            // an unknown cell.
            for (int k = 0; k < 4; ++k)
            {
                const int nx = x + dx[k];
                const int ny = y + dy[k];

                if (nx < 0 || nx >= width ||
                    ny < 0 || ny >= height)
                {
                    continue;
                }

                if (!isUnknown(nx, ny))
                    continue;

                // Midpoint of the shared free/unknown edge.
                const double lx =
                    (x + 0.5 + 0.5 * dx[k]) * resolution;
                const double ly =
                    (y + 0.5 + 0.5 * dy[k]) * resolution;

                sides.points.push_back(toMap(lx, ly));
            }
        }
    }

    markers.markers.push_back(unknown);
    markers.markers.push_back(sides);

    unknown_marker_pub_->publish(markers);
}