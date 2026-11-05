#include <mapexplorer.h>

MapExplorer::MapExplorer() : rclcpp::Node("map_explorer") {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("frontier_markers", 10);
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&MapExplorer::map_cb, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&MapExplorer::odom_cb, this, std::placeholders::_1));
}

void MapExplorer::map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_map_ = msg;
    map_received_ = true;
    std::vector<std::vector<Frontier>> clusters;
    if (is_valid_map(*latest_map_)) {
        RCLCPP_INFO(this->get_logger(), "Received a valid map.");
        clusters = detect_frontiers(*latest_map_);
        vec_frontiers = process_frontiers(clusters);

    } else {
        RCLCPP_WARN(this->get_logger(), "Received an invalid map.");
    }
}

void MapExplorer::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odom_ = msg;
    RCLCPP_INFO(this->get_logger(), "Received odometry data.");
}

bool MapExplorer::is_valid_map(const nav_msgs::msg::OccupancyGrid & map) {
    const auto & info = map.info;
    if (info.width == 0 || info.height == 0) {
        return false;
    }
    if (map.data.size() != info.width * info.height) {
        return false;
    }
    return true;
}

std::vector<std::vector<Frontier>> MapExplorer::detect_frontiers(const nav_msgs::msg::OccupancyGrid & map) {
    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);

    std::vector<std::vector<Frontier>> clusters;
    if (width < 3 || height < 3) return clusters;

    // Pass 1: mark free cells that touch unknown space
    std::vector<bool> is_frontier(static_cast<size_t>(width) * height, false);
    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            int index = y * width + x;
            if (map.data[index] != 0) continue;  // Free space only

            for (int dy = -1; dy <= 1 && !is_frontier[index]; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    if (map.data[(y + dy) * width + (x + dx)] == -1) {  // Unknown space
                        is_frontier[index] = true;
                        break;
                    }
                }
            }
        }
    }

    // Pass 2: BFS over 8-connected frontier cells; each BFS = one cluster
    std::vector<bool> visited(is_frontier.size(), false);
    for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
            int index = y * width + x;
            if (!is_frontier[index] || visited[index]) continue;

            std::vector<Frontier> cluster;
            std::queue<std::pair<int, int>> q;
            q.emplace(x, y);
            visited[index] = true;

            while (!q.empty()) {
                auto [cx, cy] = q.front();
                q.pop();
                cluster.push_back(Frontier{cx, cy});

                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        int nx = cx + dx, ny = cy + dy;
                        if (nx < 1 || nx >= width - 1 || ny < 1 || ny >= height - 1) continue;

                        int n_index = ny * width + nx;
                        if (is_frontier[n_index] && !visited[n_index]) {
                            visited[n_index] = true;
                            q.emplace(nx, ny);
                        }
                    }
                }
            }
            clusters.push_back(std::move(cluster));  // one vector per cluster
        }
    }
    return clusters;
}

std::vector<Frontier> MapExplorer::process_frontiers(const std::vector<std::vector<Frontier>> & clusters) {
    std::vector<Frontier> frontiers;
    std::vector<int> cluster_n;
    const int n = static_cast<int>(clusters.size());
    if(clusters.empty()){
        RCLCPP_INFO(this->get_logger(), "No frontiers detected.");
        publish_frontiers(frontiers, cluster_n);  // Publish empty marker array
        return clusters.empty() ? frontiers : clusters[0];  // Return empty if no clusters
    }
    for(auto cluster : clusters){
        if (n == 0 || n <= 8) continue;
        std::nth_element(cluster.begin(), cluster.begin() + n / 2, cluster.end(), [](const Frontier & a, const Frontier & b) {
            return a.x < b.x;
        });
        Frontier median = cluster[n / 2];
        frontiers.push_back(median);
        cluster_n.push_back(n);
    }
    publish_frontiers(frontiers, cluster_n);
    process_frontiers(frontiers);
    return frontiers;
}

Frontier MapExplorer::publish_frontiers(const std::vector<Frontier> & frontiers, const std::vector<int> & cluster_n) {
    if (!latest_map_) return;
    const auto & info = latest_map_->info;
    const double res = info.resolution;

    const auto & q = info.origin.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double c = std::cos(yaw), s = std::sin(yaw);

    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = latest_map_->header.frame_id;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear);

    int id = 0;
    int bigger_cluster_n = 0;
    int bigger_cluster_index = -1;
    for (size_t i = 0; i < frontiers.size(); ++i) {
        const auto & frontier = frontiers[i];
        const int cluster_size = cluster_n[i];
        if (cluster_size > bigger_cluster_n) {
            bigger_cluster_n = cluster_size;
            bigger_cluster_index = i;
        }
        RCLCPP_INFO(this->get_logger(), "Frontier %zu: cell (%d, %d), cluster size: %d", i, frontier.x, frontier.y, cluster_size);
        const double lx = (frontier.x + 0.5) * res;
        const double ly = (frontier.y + 0.5) * res;
        const double wx = info.origin.position.x + c * lx - s * ly;
        const double wy = info.origin.position.y + s * lx + c * ly;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = latest_map_->header.frame_id;
        marker.header.stamp = this->now();
        marker.ns = "frontiers";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = wx;
        marker.pose.position.y = wy;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
        marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 0.0f; marker.color.a = 1.0f;
        RCLCPP_INFO(this->get_logger(), "Frontier cell (%d, %d) -> world (%.2f, %.2f)",
                    frontier.x, frontier.y, wx, wy);
        marker_array.markers.push_back(marker);
        // Navigate to the biggest cluster frontiers[bigger_cluster_index]
    }
    RCLCPP_INFO(this->get_logger(), "Biggest cluster index: %d, size: %d", bigger_cluster_index, bigger_cluster_n);

    marker_pub_->publish(marker_array);
    return bigger_cluster_index != -1 ? frontiers[bigger_cluster_index] : Frontier{-1, -1};
}

void MapExplorer::process_frontiers(const std::vector<Frontier> & frontiers) {
    if (frontiers.empty()) {
        RCLCPP_INFO(this->get_logger(), "No frontiers to process.");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Processing %zu frontiers.", frontiers.size());
    for (const auto & frontier : frontiers) {
        RCLCPP_INFO(this->get_logger(), "Frontier at cell (%d, %d)", frontier.x, frontier.y);
    }
}