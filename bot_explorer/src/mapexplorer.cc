#include "mapexplorer.h"

MapExplorer::MapExplorer() : Node("MapExplorer"){
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&MapExplorer::MapHandler, this, _1));
}

void MapExplorer::MapHandler(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_msg){
    auto map = map_msg->data;

    for(auto cell : map){
        if(cell == 0){
            RCLCPP_INFO_ONCE(this->get_logger(), "Found Empty Space");
        } else if(cell == 1){
            RCLCPP_INFO_ONCE(this->get_logger(), "Found Occupied Space");
        } else if(cell == -1){
            RCLCPP_INFO_ONCE(this->get_logger(), "Found Boundary Space");
        } else {
            RCLCPP_INFO_ONCE(this->get_logger(), "Cell Unknow");
        }
    }
}