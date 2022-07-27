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

FusionHandler::FusionHandler(int camera_id) : Node("early_fusion_handler" + std::to_string(camera_id))
{
    RCLCPP_INFO(this->get_logger(), "Spinning Node");
    def_topics();
    RCLCPP_INFO(this->get_logger(), "Initialized topics");
    init_subscribers(camera_id);
    init_publishers(camera_id);
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
    t.jai_left_topic = pt.get<std::string>("Subscribers.jai_left_sub");
    t.jai_center_topic = pt.get<std::string>("Subscribers.jai_center_sub");
    t.jai_right_topic = pt.get<std::string>("Subscribers.jai_right_sub");

    t.left_pcl_publisher_topic = pt.get<std::string>("Publishers.left_pcl_pub");
    t.center_pcl_publisher_topic = pt.get<std::string>("Publishers.center_pcl_pub");
    t.right_pcl_publisher_topic = pt.get<std::string>("Publishers.right_pcl_pub");
    t.bb_pcl_publisher_topic = pt.get<std::string>("Publishers.bb_pcl_pub");


}

void FusionHandler::init_subscribers(int camera_id)
{
    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    
    options.callback_group = lidar_callback_group;

    pcl_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(t.pcl_subscriber_topic, qos, std::bind(&FusionHandler::lidarMsgCallback, this, std::placeholders::_1), options);

    options.callback_group = camera_callback_group;

    switch(camera_id){
        case 0 :
            jai_subscriber = this->create_subscription<turtle_interfaces::msg::BoundingBoxes>(t.jai_left_topic, qos, std::bind(&FusionHandler::cameraCallback,this,std::placeholders::_1), options);
            break;
        case 1 :
            jai_subscriber = this->create_subscription<turtle_interfaces::msg::BoundingBoxes>(t.jai_center_topic, qos, std::bind(&FusionHandler::cameraCallback,this,std::placeholders::_1), options);
            break;
        case 2 :
            jai_subscriber = this->create_subscription<turtle_interfaces::msg::BoundingBoxes>(t.jai_right_topic, qos, std::bind(&FusionHandler::cameraCallback,this,std::placeholders::_1), options);
            break;
        default : 
            RCLCPP_ERROR(this->get_logger(), "Non valid camera ID, Shuting Down...");
            break;
    }
    
}


void FusionHandler::init_publishers(int camera_id)
{
    rclcpp::SensorDataQoS Qos;

    switch(camera_id){
        case 0:
            pcl_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(t.left_pcl_publisher_topic, Qos);
            break;
        case 1:
            pcl_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(t.center_pcl_publisher_topic, Qos);
            break;
        case 2:
            pcl_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(t.right_pcl_publisher_topic, Qos);
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Non valid camera ID, Shuting Down...");
    }
    bb_pcl_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(t.bb_pcl_publisher_topic,Qos);

    coneDistancesMsg.header.frame_id = "os1";
    boundingBoxPclMsg.header.frame_id = "os1";

    coneDistancesMsg.height = 1;
    boundingBoxPclMsg.height = 1;

    coneDistancesMsg.is_bigendian = false;
    boundingBoxPclMsg.is_bigendian = false;

    coneDistancesMsg.point_step = 12;
    boundingBoxPclMsg.point_step = 12;
    // coneDistancesMsg.point_step = 16;

    // coneDistancesMsg.fields.resize(4); //x, y, z, rgb
     
    coneDistancesMsg.fields.resize(3);
    boundingBoxPclMsg.fields.resize(3);

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

    // coneDistancesMsg.fields[3].name = "rgb";
    // coneDistancesMsg.fields[3].offset = 12;
    // coneDistancesMsg.fields[3].datatype = 7;
    // coneDistancesMsg.fields[3].count = 1;

    boundingBoxPclMsg.fields[0].name = "x";
    boundingBoxPclMsg.fields[0].offset = 0;
    boundingBoxPclMsg.fields[0].datatype = 7;
    boundingBoxPclMsg.fields[0].count = 1;

    boundingBoxPclMsg.fields[1].name = "y";
    boundingBoxPclMsg.fields[1].offset = 4;
    boundingBoxPclMsg.fields[1].datatype = 7;
    boundingBoxPclMsg.fields[1].count = 1;

    boundingBoxPclMsg.fields[2].name = "z";
    boundingBoxPclMsg.fields[2].offset = 8;
    boundingBoxPclMsg.fields[2].datatype = 7;
    boundingBoxPclMsg.fields[2].count = 1;
    
}

void FusionHandler::lidarMsgCallback(sensor_msgs::msg::PointCloud2 pcl_msg)
{
    fusion_mutex.lock();
    this->latest_pcl = pcl_msg;
    fusion_mutex.unlock();

    // std::cout << "Save latest PointCloud message"<<std::endl;
    lidar_flag = true;
}

void FusionHandler::cameraCallback(const turtle_interfaces::msg::BoundingBoxes cam_msg)
{
    // std::cout<<"Inside Camera Callback"<<std::endl;
    // std::cout<<"Camera identifier : "<<(int)cam_msg.camera<<std::endl;

    if(lidar_flag && cam_msg.x.size() != 0){
        fusion_mutex.lock_shared();
        sensor_msgs::msg::PointCloud2 fusion_pcl = this->latest_pcl;
        
        fusion(fusion_pcl, cam_msg);
        publishCones();
        publishBBPcl();
        
        fusion_mutex.unlock_shared();
    }

}

void Fusion::fusion(sensor_msgs::msg::PointCloud2 pcl_msg , turtle_interfaces::msg::BoundingBoxes cam_msg)
{

    int camera_id = (int)cam_msg.camera;
    set_lidar_XYZ(pcl_msg);
    read_intrinsic_params(camera_id);
    calculate_transformation_matrix(camera_id);
    calculate_pixel_points();
    find_inside_bounding_boxes(cam_msg);
}

void FusionHandler::publishCones()
{
    
    // Matrix4Xf pcl_msg = get_pcl_xyz();
    Matrix3Xf pcl_msg ;

    pcl_msg = get_pcl_xyz();

    coneDistancesMsg.width = pcl_msg.cols();
    coneDistancesMsg.row_step = coneDistancesMsg.width * coneDistancesMsg.point_step;

    coneDistancesMsg.data.resize(coneDistancesMsg.row_step);

    
    uint8_t* ptr = coneDistancesMsg.data.data();
    coneDistancesMsg.is_dense = false;
   
    for (int i = 0; i < pcl_msg.cols(); i++){

        *((float*)(ptr + i*coneDistancesMsg.point_step)) = pcl_msg(0,i);

        *((float*)(ptr + i*coneDistancesMsg.point_step + 4)) = pcl_msg(1,i);

        *((float*)(ptr + i*coneDistancesMsg.point_step + 8)) = pcl_msg(2,i);

        // *((float*)(ptr + i*coneDistancesMsg.point_step + 12)) = pcl_msg(3,i);
    }
    
    pcl_publisher->publish(coneDistancesMsg);
    // std::cout<<"MESSAGE PUBLISHED!!!!"<<std::endl;
}

void FusionHandler::publishBBPcl(){

    Matrix3Xf bb_pcl_msg;


    bb_pcl_msg = get_bb_pcl();


    boundingBoxPclMsg.width = bb_pcl_msg.cols();
    boundingBoxPclMsg.row_step = boundingBoxPclMsg.width * boundingBoxPclMsg.point_step;

    boundingBoxPclMsg.data.resize(boundingBoxPclMsg.row_step);

    
    uint8_t* ptr = boundingBoxPclMsg.data.data();
    // boundingBoxPclMsg.is_dense = false;
   
    for (int i = 0; i < bb_pcl_msg.cols(); i++){

        *((float*)(ptr + i*boundingBoxPclMsg.point_step)) = bb_pcl_msg(0,i);

        *((float*)(ptr + i*boundingBoxPclMsg.point_step + 4)) = bb_pcl_msg(1,i);

        *((float*)(ptr + i*boundingBoxPclMsg.point_step + 8)) = bb_pcl_msg(2,i);

        // *((float*)(ptr + i*coneDistancesMsg.point_step + 12)) = pcl_msg(3,i);
    }
    
    bb_pcl_publisher->publish(boundingBoxPclMsg);
    // std::cout<<"MESSAGE PUBLISHED!!!!"<<std::endl;

}