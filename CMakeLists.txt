cmake_minimum_required(VERSION 3.5)
project(lio_sam)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -pthread -fopenmp")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
set(OpenCV_STATIC ON)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenMP REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  PCL
  OpenCV
  livox_ros_driver2
)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/CloudInfo.msg" "srv/SaveMap.srv" DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

# add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)

# add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)

add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration gtsam)

add_executable(${PROJECT_NAME}_mapOptimization 
  src/mapOptmization.cpp 
  include/sc/Scancontext.cpp
)
target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam)

# ament_target_dependencies(${PROJECT_NAME}_featureExtraction ${dependencies})
# ament_target_dependencies(${PROJECT_NAME}_imageProjection ${dependencies})
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration ${dependencies})
ament_target_dependencies(${PROJECT_NAME}_mapOptimization ${dependencies})

# rosidl_target_interfaces(${PROJECT_NAME}_featureExtraction ${PROJECT_NAME} "rosidl_typesupport_cpp")
# rosidl_target_interfaces(${PROJECT_NAME}_imageProjection ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(${PROJECT_NAME}_imuPreintegration ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(${PROJECT_NAME}_mapOptimization ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# install(TARGETS ${PROJECT_NAME}_featureExtraction DESTINATION lib/${PROJECT_NAME})
# install(TARGETS ${PROJECT_NAME}_imageProjection DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_imuPreintegration DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_mapOptimization DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()
