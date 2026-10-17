#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include "mapexplorer.h"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapExplorer>());
    rclcpp::shutdown();
    return 0;
}