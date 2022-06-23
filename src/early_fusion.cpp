#include <memory>
#include <stdio.h>

#include "eigen3/Eigen/Core"
#include "yaml-cpp/yaml.h"

#include "FusionHandler.hpp"

#define LIDAR_TOPIC "ouster/RawPointcloud"
#define LEFT_CAMERA_TOPIC "/jai_left_boundingboxes"
#define CENTER_CAMERA_TOPIC "/jai_center_boundingboxes"
#define RIGHT_CAMERA_TOPIC "/jai_right_boundingboxes"

using namespace Eigen;

Matrix3f camera_matrix;
Matrix<float, 3, 4> transformation_matrix;

FusionHandler::FusionHandler() : Node("early_fusion_handler")
{
    init_subscribers();
    init_publishers();
}

FusionHandler::~FusionHandler()
{
}

void FusionHandler::init_subscribers()
{
    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    
    pcl_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(LIDAR_TOPIC, qos, std::bind(&FusionHandler::lidarMsgCallback, this, std::placeholders::_1));
    
    jai_left_subscriber = this->create_subscription<turtle_interfaces::msg::BoundingBoxes>(LEFT_CAMERA_TOPIC, qos, std::bind(&FusionHandler::cameraCallback,this,std::placeholders::_1, 0));
    jai_center_subscriber = this->create_subscription<turtle_interfaces::msg::BoundingBoxes>(CENTER_CAMERA_TOPIC, qos, std::bind(&FusionHandler::cameraCallback,this,std::placeholders::_1, 1));
    jai_right_subscriber = this->create_subscription<turtle_interfaces::msg::BoundingBoxes>(RIGHT_CAMERA_TOPIC, qos, std::bind(&FusionHandler::cameraCallback,this,std::placeholders::_1, 2));

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

void FusionHandler::cameraCallback(turtle_interfaces::msg::BoundingBoxes cam_msg, int cam_id)
{
    std::cout<<"Inside Camera Callback"<<std::endl;
    std::cout<<"Camera identifier : "<<cam_id<<std::endl;

    fusion_mutex.lock_shared();
    auto fusion_pcl = this->latest_pcl;
    fusion_mutex.unlock_shared();

    fusion(fusion_pcl, cam_msg, cam_id);

    publishCones();
}

void Fusion::fusion(sensor_msgs::msg::PointCloud2 pcl_msg , turtle_interfaces::msg::BoundingBoxes cam_msg , int cam_id)
{
    set_lidar_XYZ(pcl_msg);
    read_intrinsic_params(cam_id);
    calculate_transformation_matrix(cam_id);
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