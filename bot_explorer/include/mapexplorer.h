#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
using std::placeholders::_1;

class MapExplorer : public rclcpp::Node{
public:
    MapExplorer();
private:
    void MapHandler(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_msg);
    void ExplorerPointPub(double x, double y);
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr explore_point_pub_;
};