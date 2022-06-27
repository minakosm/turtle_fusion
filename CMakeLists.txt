cmake_minimum_required(VERSION 3.8)
project(turtle_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtle_interfaces REQUIRED)
find_package(turtle_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rcutils REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(ament_index_cpp)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${OpenCV_INCLUDE_DIRS}
                    ${YAML_CPP_INCLUDE_DIRS})

add_library(fusion src/early_fusion.cpp)
target_link_libraries(fusion Eigen3::Eigen)
target_link_libraries(fusion ${YAML_CPP_LIBRARIES})
target_link_libraries(fusion ${OpenCV_LIBRARIES})
ament_target_dependencies(fusion rclcpp std_msgs sensor_msgs ament_index_cpp tf2_ros turtle_interfaces)

add_executable(camera_lidar_fusion src/main.cpp)
target_link_libraries(camera_lidar_fusion fusion)
ament_target_dependencies(camera_lidar_fusion rosbag2_cpp rosbag2_storage rcutils sensor_msgs)

install(DIRECTORY
        settings
        DESTINATION share/${PROJECT_NAME})

install(TARGETS camera_lidar_fusion
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
