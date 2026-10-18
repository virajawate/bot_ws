#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include "mapexplorer.h"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapExplorer>();
    while(rclcpp::ok()){
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}