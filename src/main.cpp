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
    rclcpp::executors::MultiThreadedExecutor executor;

    rclcpp::Node::SharedPtr node = std::make_shared<FusionHandler>();
    executor.add_node(node);

    executor.spin();
    // rclcpp::spin(std::make_shared<FusionHandler>());
    rclcpp::shutdown();
    return 0;
}