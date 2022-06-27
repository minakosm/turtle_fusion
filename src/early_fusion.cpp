#include <memory>
#include <stdio.h>

#include "eigen3/Eigen/Core"
#include "yaml-cpp/yaml.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include "turtle_fusion/FusionHandler.hpp"

using namespace Eigen;

Matrix3f camera_matrix;
Matrix<float, 3, 4> transformation_matrix;

FusionHandler::FusionHandler() : Node("early_fusion_handler")
{
    RCLCPP_INFO(this->get_logger(), "Spinning Node");
    def_topics();
    RCLCPP_INFO(this->get_logger(), "Initialized topics");
    init_subscribers();
    init_publishers();
    RCLCPP_INFO(this->get_logger(), "Initialized publisers/subscribers, waiting for callbacks");
}

FusionHandler::~FusionHandler()
{
}

void FusionHandler::def_topics()
{
    std::string filename = ament_index_cpp::get_package_share_directory("turtle_fusion")
                           + "/settings/topic_settings.ini";

    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(filename, pt);

    t.pcl_subscriber_topic = pt.get<std::string>("Subscribers.pcl_sub");

    t.pcl_publisher_topic = pt.get<std::string>("Publishers.pcl_pub");
    t.jai_left_topic = pt.get<std::string>("Publishers.jai_left_pub");
    t.jai_center_topic = pt.get<std::string>("Publishers.jai_center_pub");
    t.jai_right_topic = pt.get<std::string>("Publishers.jai_right_pub");

}

void FusionHandler::init_subscribers()
{
    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    
    pcl_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(t.pcl_subscriber_topic, qos, std::bind(&FusionHandler::lidarMsgCallback, this, std::placeholders::_1));

    // std::function<void(const turtle_interfaces::msg::BoundingBoxes msg)> jai_left_subscriber = std::bind(&FusionHandler::cameraCallback, this, std::placeholders::_1, 0);
    // std::function<void(const turtle_interfaces::msg::BoundingBoxes msg)> jai_center_subscriber = std::bind(&FusionHandler::cameraCallback, this, std::placeholders::_1, 1);
    // std::function<void(const turtle_interfaces::msg::BoundingBoxes msg)> jai_right_subscriber = std::bind(&FusionHandler::cameraCallback, this, std::placeholders::_1, 2);

    jai_left_subscriber = this->create_subscription<turtle_interfaces::msg::BoundingBoxes>(t.jai_left_topic, qos, std::bind(&FusionHandler::cameraCallback,this,std::placeholders::_1));
    jai_center_subscriber = this->create_subscription<turtle_interfaces::msg::BoundingBoxes>(t.jai_center_topic, qos, std::bind(&FusionHandler::cameraCallback,this,std::placeholders::_1));
    jai_right_subscriber = this->create_subscription<turtle_interfaces::msg::BoundingBoxes>(t.jai_right_topic, qos, std::bind(&FusionHandler::cameraCallback,this,std::placeholders::_1));

}


void FusionHandler::init_publishers()
{
    rclcpp::SensorDataQoS Qos;

    pcl_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/fusion/coneDistances", Qos);

    coneDistancesMsg.header.frame_id = "os1";
    coneDistancesMsg.fields.resize(4); //x, y, z
        
    coneDistancesMsg.fields[0].name = "x";
    coneDistancesMsg.fields[0].offset = 0;
    coneDistancesMsg.fields[0].datatype = 7;
    coneDistancesMsg.fields[0].count = 1;

    coneDistancesMsg.fields[1].name = "y";
    coneDistancesMsg.fields[1].offset = 4;
    coneDistancesMsg.fields[1].datatype = 7;
    coneDistancesMsg.fields[1].count = 1;

    coneDistancesMsg.fields[2].name = "z";
    coneDistancesMsg.fields[2].offset = 8;
    coneDistancesMsg.fields[2].datatype = 7;
    coneDistancesMsg.fields[2].count = 1;

    coneDistancesMsg.fields[3].name = "t";
    coneDistancesMsg.fields[3].offset = 12;
    coneDistancesMsg.fields[3].datatype = 6;
    coneDistancesMsg.fields[3].count = 1;
    /*
        ADD ConeDistances attributes
    */
    
}

void FusionHandler::lidarMsgCallback(sensor_msgs::msg::PointCloud2 pcl_msg)
{
    fusion_mutex.lock();
    this->latest_pcl = pcl_msg;
    fusion_mutex.unlock();

    std::cout << "Save latest PointCloud message"<<std::endl;
}

void FusionHandler::cameraCallback(const turtle_interfaces::msg::BoundingBoxes cam_msg)
{
    std::cout<<"Inside Camera Callback"<<std::endl;
    std::cout<<"Camera identifier : "<<cam_msg.camera<<std::endl;

    fusion_mutex.lock_shared();
    auto fusion_pcl = this->latest_pcl;
    fusion_mutex.unlock_shared();

    fusion(fusion_pcl, cam_msg);

    publishCones();
}

void Fusion::fusion(sensor_msgs::msg::PointCloud2 pcl_msg , turtle_interfaces::msg::BoundingBoxes cam_msg)
{
    set_lidar_XYZ(pcl_msg);
    read_intrinsic_params(cam_msg.camera);
    calculate_transformation_matrix(cam_msg.camera);
    calculate_pixel_points();

    find_inside_bounding_boxes(cam_msg);
}

void FusionHandler::publishCones()
{
    uint8_t* ptr = coneDistancesMsg.data.data();

    coneDistancesMsg.width = get_pcl_xyz().cols();
    coneDistancesMsg.height = 1;
    coneDistancesMsg.is_bigendian = false;
    coneDistancesMsg.point_step = 16;
    coneDistancesMsg.row_step = coneDistancesMsg.width * coneDistancesMsg.point_step;
    coneDistancesMsg.is_dense = true;

    coneDistancesMsg.data.resize(coneDistancesMsg.point_step * coneDistancesMsg.width);
    
    for (int i = 0; i < get_pcl_xyz().cols(); i++){
       *((float*)(ptr + i*coneDistancesMsg.point_step)) = get_pcl_xyz()(0,i);
       *((float*)(ptr + i*coneDistancesMsg.point_step + 4)) = get_pcl_xyz()(1,i);
       *((float*)(ptr + i*coneDistancesMsg.point_step + 8)) = get_pcl_xyz()(2,i);
    }
    
    pcl_publisher->publish(coneDistancesMsg);

}