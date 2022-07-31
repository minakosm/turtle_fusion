#include "early_fusion.cpp"
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp" 

/**
 * TODO BEFORE PRODUCTION : 
 *  1) DELETE EVERY LOGGER 
 *  2) DELETE EVERY STD::COUT
 */

int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;

    rclcpp::Node::SharedPtr node_left = std::make_shared<FusionHandler>(0);
    rclcpp::Node::SharedPtr node_center = std::make_shared<FusionHandler>(1);
    rclcpp::Node::SharedPtr node_right = std::make_shared<FusionHandler>(2);

    executor.add_node(node_left);
    executor.add_node(node_center);
    executor.add_node(node_right);

    executor.spin();
    // rclcpp::spin(std::make_shared<FusionHandler>());
    rclcpp::shutdown();
    return 0;
}