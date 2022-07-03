#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "turtle_interfaces/msg/bounding_boxes.hpp"

#include <shared_mutex>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "turtle_fusion/Fusion.hpp"

#ifndef CAMERA_N
#define CAMERA_N 3
#endif

#ifndef PI
#define PI 3.14159
#endif


class FusionHandler : public rclcpp::Node, private Fusion
{
private:

    struct Topics{
        std::string pcl_subscriber_topic;
        std::string pcl_publisher_topic;
        std::string jai_left_topic;
        std::string jai_center_topic;
        std::string jai_right_topic;
    };

    Topics t;

    rclcpp::SubscriptionOptions options;
    rclcpp::CallbackGroup::SharedPtr camera_callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::CallbackGroup::SharedPtr lidar_callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber;
    // message_filters::Subscriber<sensor_msgs::msg::PointCloud2> *pcl_subscriber;

    rclcpp::Subscription<turtle_interfaces::msg::BoundingBoxes>::SharedPtr jai_left_subscriber;
    rclcpp::Subscription<turtle_interfaces::msg::BoundingBoxes>::SharedPtr jai_center_subscriber;
    rclcpp::Subscription<turtle_interfaces::msg::BoundingBoxes>::SharedPtr jai_right_subscriber;

    // message_filters::Subscriber<turtle_interfaces::msg::BoundingBoxes> *jai_left_subscriber;
    // message_filters::Subscriber<turtle_interfaces::msg::BoundingBoxes> *jai_center_subscriber;
    // message_filters::Subscriber<turtle_interfaces::msg::BoundingBoxes> *jai_right_subscriber;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher;
    sensor_msgs::msg::PointCloud2 coneDistancesMsg; // x y z rgb t 

    std::shared_timed_mutex fusion_mutex;
    bool lidar_flag;
    sensor_msgs::msg::PointCloud2 latest_pcl;



public: 
    FusionHandler();
    ~FusionHandler();

    void def_topics();
    void init_publishers();
    void init_subscribers();

    void lidarMsgCallback(const sensor_msgs::msg::PointCloud2);
    void cameraCallback(const turtle_interfaces::msg::BoundingBoxes);

    void publishCones();

};