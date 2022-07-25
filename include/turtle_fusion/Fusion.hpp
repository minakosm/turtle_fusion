#include <stdio.h>
#include <iostream>

#include "eigen3/Eigen/Eigen"
#include "eigen3/Eigen/Core"

#include "yaml-cpp/yaml.h"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "turtle_interfaces/msg/bounding_boxes.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"

#include "rclcpp/rclcpp.hpp"


using namespace Eigen;

#define IMAGE_WIDTH 2560
#define IMAGE_HEIGHT 2048

class Fusion
{
private:
MatrixXf lidar_xyz;                             // LiDAR 3D points

Matrix<float, 3, 4> transformation_matrix;      // 3x4 [R | t] matrix

std::vector<cv::Point2f> px;                    // image frame pixel points 

Matrix3f intrinsic_K;                           // Camera Matrix 
Matrix<float, 1, 5> intrinsic_D;                // Distortion Coefficients    

// Matrix4Xf pcl_xyz;                              // Pcl message to be pushed
Matrix3Xf pcl_xyz;
// float cone_color;
Matrix3Xf bb_pcl;

public:
    void set_lidar_XYZ(sensor_msgs::msg::PointCloud2);
    void read_intrinsic_params(int);
    void calculate_transformation_matrix(int);

    void calculate_pixel_points();
    void find_inside_bounding_boxes(turtle_interfaces::msg::BoundingBoxes);
    void extract_distance(std::vector<float>, std::vector<float>, std::vector<float>, int);
    
    void assign_bb_pcl(std::vector<float>);

    void fusion(sensor_msgs::msg::PointCloud2, turtle_interfaces::msg::BoundingBoxes);

    MatrixXf get_lidar_xyz(){return lidar_xyz;}
    std::vector<cv::Point2f> get_px(){return px;}
    // Matrix4Xf get_pcl_xyz(){return pcl_xyz;}
    Matrix3Xf get_pcl_xyz(){return pcl_xyz;}
    Matrix3Xf get_bb_pcl(){return bb_pcl;}
};

void Fusion::set_lidar_XYZ(sensor_msgs::msg::PointCloud2 pcl_msg)
{
    uint8_t* ptr = pcl_msg.data.data();
    int pcl_size = pcl_msg.data.size()/pcl_msg.point_step;

    std::cout<<"PCL SIZE = "<<pcl_size<<std::endl<<std::endl;
    // resize 4xN (x(i) y(i) z(i) 1)
    lidar_xyz.resize(4,pcl_size);

    for(int i=0; i<pcl_size; i++){

        lidar_xyz(0,i) = *((float*)(ptr + i*pcl_msg.point_step));       // X
        lidar_xyz(1,i) = *((float*)(ptr + i*pcl_msg.point_step + 4));   // Y
        lidar_xyz(2,i) = *((float*)(ptr + i*pcl_msg.point_step + 8));   // Z
        lidar_xyz(3,i) = 1;                                             // Homogenous 1 
    }

}

void Fusion::read_intrinsic_params(int camera_id)
{
    std::string package_share_path = ament_index_cpp::get_package_share_directory("turtle_calibration");
    std::string intrinsic_filename = package_share_path + "/settings/intrinsic_params" + std::to_string(camera_id) + ".yaml";

    YAML::Node yaml_root = YAML::LoadFile(intrinsic_filename);

    for(YAML::const_iterator it = yaml_root.begin(); it!=yaml_root.end(); it++){
        
        const std::string &key = it->first.as<std::string>();

        YAML::Node attributes = it->second;
        int rows = attributes["rows"].as<int>();
        int cols = attributes["cols"].as<int>();

        const std::vector<float> data = attributes["data"].as<std::vector<float>>();

        for(int i=0; i<rows; i++){
            for(int j=0; j<cols; j++){
                if(key =="K"){
                    intrinsic_K(i,j) = data[i*cols + j];
                } else if(key == "D"){
                    intrinsic_D(j) = data[j];
                }else{
                    std::cout<<"Invalid Yaml Key"<<std::endl;
                    break;
                }
            }
        }

    }
}

void Fusion::calculate_transformation_matrix(int camera_id){
    float roll, pitch, yaw;
    float t_x, t_y, t_z;

    std::string filename = ament_index_cpp::get_package_share_directory("turtle_calibration") + 
                           "/settings/lidar_camera_extrinsic.ini";

    Matrix3f rotation_matrix;
    Vector3f translation_vector;

    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(filename, pt);

    std::string label_roll = "Camera"+std::to_string(camera_id)+".roll";
    std::string label_pitch = "Camera"+std::to_string(camera_id)+".pitch";
    std::string label_yaw = "Camera"+std::to_string(camera_id)+".yaw";

    std::string label_t_x = "Camera"+std::to_string(camera_id)+".t_x";
    std::string label_t_y = "Camera"+std::to_string(camera_id)+".t_y";
    std::string label_t_z = "Camera"+std::to_string(camera_id)+".t_z";
    
    roll = pt.get<float>(label_roll);
    pitch = pt.get<float>(label_pitch);
    yaw = pt.get<float>(label_yaw);

    t_x = pt.get<float>(label_t_x);
    t_y = pt.get<float>(label_t_y);
    t_z = pt.get<float>(label_t_z);

    translation_vector << t_x, 
                          t_y, 
                          t_z;

    rotation_matrix = AngleAxisf(roll, Vector3f::UnitX())
                      *AngleAxisf(pitch, Vector3f::UnitY())
                      *AngleAxisf(yaw, Vector3f::UnitZ());

    transformation_matrix.resize(3,4);
    transformation_matrix << rotation_matrix, translation_vector;
}

void Fusion::calculate_pixel_points()
{

    MatrixXf pixel_homeogenous_points;
    pixel_homeogenous_points.resize(3, lidar_xyz.cols());
    pixel_homeogenous_points = intrinsic_K * transformation_matrix * lidar_xyz;

    px.resize(pixel_homeogenous_points.cols());
    for(int i=0; i<pixel_homeogenous_points.cols(); i++){
        pixel_homeogenous_points(0,i) = pixel_homeogenous_points(0,i)/pixel_homeogenous_points(2,i);
        pixel_homeogenous_points(1,i) = pixel_homeogenous_points(1,i)/pixel_homeogenous_points(2,i);
        
        px[i].x = pixel_homeogenous_points(0,i);
        px[i].y = pixel_homeogenous_points(1,i);

    }
}

void Fusion::find_inside_bounding_boxes(turtle_interfaces::msg::BoundingBoxes cam_msg)
{
    std::vector<float> x_buf, y_buf, z_buf, indexes;
    int counter = 0;
    // pcl_xyz.resize(4,cam_msg.x.size());
    pcl_xyz.resize(3,cam_msg.x.size());

    // std::cout<<"FOUND "<<cam_msg.x.size()<< " BOUNDING BOXES "<<std::endl;
    for(int i=0; i<cam_msg.x.size(); i++){

        // cone_color = cam_msg.color[i];
        cv::Rect2f bounding_box(cam_msg.x[i] * IMAGE_WIDTH, cam_msg.y[i] * IMAGE_HEIGHT, cam_msg.w[i] * IMAGE_WIDTH, cam_msg.h[i] * IMAGE_HEIGHT);

        for(int j=0; j<px.size(); j++){
            if(px[j].inside(bounding_box)){
                x_buf.push_back(lidar_xyz(0,j));    //x(j)
                y_buf.push_back(lidar_xyz(1,j));    //y(j)
                z_buf.push_back(lidar_xyz(2,j));    //z(j) 

                counter++;
                indexes.push_back(j);
                // bb_pcl.resize(3,counter);
                // bb_pcl(0,counter) = lidar_xyz(0,j);
                // bb_pcl(1,counter) = lidar_xyz(1,j);
                // bb_pcl(2,counter) = lidar_xyz(2,j);
            }


        }
        if(x_buf.size() == 0){
            // std::cout<<"NO POINTS FOUND FOR BOUNDING BOX "<<i<<std::endl;   
            continue;
        }
        extract_distance(x_buf, y_buf, z_buf, i);

        x_buf.clear();
        y_buf.clear();
        z_buf.clear();
    }
    assign_bb_pcl(indexes);
    indexes.clear();
}

void Fusion::assign_bb_pcl(std::vector<float> id){
    bb_pcl.resize(3,id.size());
    for (int i=0; i<id.size(); i++){
        bb_pcl(0,i) = lidar_xyz(0,id[i]);
        bb_pcl(1,i) = lidar_xyz(1,id[i]);
        bb_pcl(2,i) = lidar_xyz(2,id[i]);
    }
}


void Fusion::extract_distance(std::vector<float> v_x, std::vector<float> v_y, std::vector<float> v_z, int bounding_box_id)
{
    float mean_x = 0;
    float mean_y = 0; 
    float mean_z = 0;

    for(int i=0; i<v_x.size(); i++){
        mean_x = mean_x + v_x[i];
        mean_y = mean_y + v_y[i];
        mean_z = mean_z + v_z[i];
    }
    mean_x = mean_x / v_x.size();
    mean_y = mean_y / v_y.size();
    mean_z = mean_z / v_z.size();

    // pcl_xyz(0,bounding_box_id) = mean_x;
    // pcl_xyz(1,bounding_box_id) = mean_y;
    // pcl_xyz(2,bounding_box_id) = mean_z;

    // pcl_xyz(3,bounding_box_id) = cone_color;


    float r = 10000;
    int flag = 0;
    for(int i=0; i<v_x.size(); i++){
        float tmp = sqrt(pow(v_x[i],2) + pow(v_y[i],2) + pow(v_z[i],2));
        if(tmp<r){
            r = tmp;
            flag = i;
        }
    }

    pcl_xyz(0,bounding_box_id) = (mean_x + v_x[flag]) / 2;
    pcl_xyz(1,bounding_box_id) = (mean_y + v_y[flag]) / 2;
    pcl_xyz(2,bounding_box_id) = (mean_z + v_z[flag]) / 2;

}