#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

using std::placeholders::_1;

class MapExplorer : public rclcpp::Node{
public:
    MapExplorer();
private:
    void MapHandler(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_msg);
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};