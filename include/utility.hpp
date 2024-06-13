#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "livox_ros_driver2/msg/custom_msg.hpp"

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <string>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;

using CloudType = pcl::PointCloud<PointType>;
using CloudPtr = pcl::PointCloud<PointType>::Ptr;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

enum class SensorType { VELODYNE, OUSTER, LIVOX };

#include <chrono>
#define GET_TIME() std::chrono::high_resolution_clock::now()
#define GET_USED(t2, t1) std::chrono::duration<double>(t2 - t1).count()

class ParamServer : public rclcpp::Node {
public:
    std::string robot_id;

    // Topics
    string pointCloudTopic, imuTopic;
    string odomTopic, gpsTopic;

    // Frames
    string lidarFrame, baselinkFrame, odometryFrame, mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold, poseCovThreshold;

    // Save pcd
    bool savePCD, saveKeyframeMap;
    string savePCDDirectory;

    // Lidar Sensor Configuration
    SensorType sensor = SensorType::LIVOX;
    int N_SCAN, Horizon_SCAN;
    int downsampleRate;
    float lidarMinRange, lidarMaxRange;

    // IMU
    float imuAccNoise, imuGyrNoise;
    float imuAccBiasN, imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV, extRPYV, extTransV;
    Eigen::Matrix3d extRot, extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold, surfThreshold;
    int edgeFeatureMinValidNum, surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize, mappingSurfLeafSize;

    float z_tollerance, rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold;
    float surroundingkeyframeAddingAngleThreshold;
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;

    // Loop closure
    bool loopClosureEnableFlag;
    float loopClosureFrequency;
    int surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer() : Node("ParamServerNode") {
        declare_and_get_parameter<std::string>("pointCloudTopic", pointCloudTopic, "points");
        declare_and_get_parameter<std::string>("imuTopic", imuTopic, "imu/data");
        declare_and_get_parameter<std::string>("odomTopic", odomTopic, "lio_sam/odometry/imu");
        declare_and_get_parameter<std::string>("gpsTopic", gpsTopic, "lio_sam/odometry/gps");

        declare_and_get_parameter<std::string>("lidarFrame", lidarFrame, "laser_data_frame");
        declare_and_get_parameter<std::string>("baselinkFrame", baselinkFrame, "base_link");
        declare_and_get_parameter<std::string>("odometryFrame", odometryFrame, "odom");
        declare_and_get_parameter<std::string>("mapFrame", mapFrame, "map");

        declare_and_get_parameter<bool>("useImuHeadingInitialization", useImuHeadingInitialization, false);
        declare_and_get_parameter<bool>("useGpsElevation", useGpsElevation, false);
        declare_and_get_parameter<float>("gpsCovThreshold", gpsCovThreshold, 2.0);
        declare_and_get_parameter<float>("poseCovThreshold", poseCovThreshold, 25.0);

        declare_and_get_parameter<bool>("savePCD", savePCD, false);
        declare_and_get_parameter<bool>("saveKeyframeMap", saveKeyframeMap, false);
        declare_and_get_parameter<std::string>("savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

        std::string sensorStr;
        declare_and_get_parameter<std::string>("sensor", sensorStr, "ouster");
        if (sensorStr == "velodyne") {
            sensor = SensorType::VELODYNE;
        } else if (sensorStr == "ouster") {
            sensor = SensorType::OUSTER;
        } else if (sensorStr == "livox") {
            sensor = SensorType::LIVOX;
        } else {
            RCLCPP_ERROR_STREAM(get_logger(), "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox'): " << sensorStr);
            rclcpp::shutdown();
        }

        declare_and_get_parameter<int>("N_SCAN", N_SCAN, 64);
        declare_and_get_parameter<int>("Horizon_SCAN", Horizon_SCAN, 512);
        declare_and_get_parameter<int>("downsampleRate", downsampleRate, 1);
        declare_and_get_parameter<float>("lidarMinRange", lidarMinRange, 5.5);
        declare_and_get_parameter<float>("lidarMaxRange", lidarMaxRange, 1000.0);

        declare_and_get_parameter<float>("imuAccNoise", imuAccNoise, 9e-4);
        declare_and_get_parameter<float>("imuGyrNoise", imuGyrNoise, 1.6e-4);
        declare_and_get_parameter<float>("imuAccBiasN", imuAccBiasN, 5e-4);
        declare_and_get_parameter<float>("imuGyrBiasN", imuGyrBiasN, 7e-5);
        declare_and_get_parameter<float>("imuGravity", imuGravity, 9.80511);
        declare_and_get_parameter<float>("imuRPYWeight", imuRPYWeight, 0.01);

        double ida[] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        std::vector<double> id(ida, std::end(ida));
        declare_parameter("extrinsicRot", id);
        get_parameter("extrinsicRot", extRotV);
        declare_parameter("extrinsicRPY", id);
        get_parameter("extrinsicRPY", extRPYV);
        double zea[] = {0.0, 0.0, 0.0};
        std::vector<double> ze(zea, std::end(zea));
        declare_parameter("extrinsicTrans", ze);
        get_parameter("extrinsicTrans", extTransV);

        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        declare_and_get_parameter<float>("edgeThreshold", edgeThreshold, 1.0);
        declare_and_get_parameter<float>("surfThreshold", surfThreshold, 0.1);
        declare_and_get_parameter<int>("edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        declare_and_get_parameter<int>("surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        declare_and_get_parameter<float>("odometrySurfLeafSize", odometrySurfLeafSize, 0.4);
        declare_and_get_parameter<float>("mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        declare_and_get_parameter<float>("mappingSurfLeafSize", mappingSurfLeafSize, 0.4);

        declare_and_get_parameter<float>("z_tollerance", z_tollerance, 1000.0);
        declare_and_get_parameter<float>("rotation_tollerance", rotation_tollerance, 1000.0);

        declare_and_get_parameter<int>("numberOfCores", numberOfCores, 4);
        declare_and_get_parameter<double>("mappingProcessInterval", mappingProcessInterval, 0.15);

        declare_and_get_parameter<float>("surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        declare_and_get_parameter<float>("surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        declare_and_get_parameter<float>("surroundingKeyframeDensity", surroundingKeyframeDensity, 2.0);
        declare_and_get_parameter<float>("surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        declare_and_get_parameter<bool>("loopClosureEnableFlag", loopClosureEnableFlag, true);
        declare_and_get_parameter<float>("loopClosureFrequency", loopClosureFrequency, 1.0);
        declare_and_get_parameter<int>("surroundingKeyframeSize", surroundingKeyframeSize, 50);
        declare_and_get_parameter<float>("historyKeyframeSearchRadius", historyKeyframeSearchRadius, 15.0);
        declare_and_get_parameter<float>("historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        declare_and_get_parameter<int>("historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        declare_and_get_parameter<float>("historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        declare_and_get_parameter<float>("globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1000.0);
        declare_and_get_parameter<float>("globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        declare_and_get_parameter<float>("globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        usleep(100);
    }

    template <typename T>
    void declare_and_get_parameter(const std::string& name, T& variable, const T& default_value) {
        this->declare_parameter<T>(name, default_value);
        this->get_parameter(name, variable);
    }

    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in) {
        sensor_msgs::msg::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);

        acc *= imuGravity;

        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = extQRPY;  // q_from * extQRPY;
        q_final.normalize();
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x() * q_final.x() + q_final.y() * q_final.y() + q_final.z() * q_final.z() + q_final.w() * q_final.w()) < 0.1) {
            RCLCPP_ERROR(get_logger(), "Invalid quaternion, please use a 9-axis IMU!");
            rclcpp::shutdown();
        }

        return imu_out;
    }
};

inline sensor_msgs::msg::PointCloud2 publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub,
                                                  pcl::PointCloud<PointType>::Ptr thisCloud, rclcpp::Time thisStamp, std::string thisFrame) {
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->get_subscription_count() != 0) thisPub->publish(tempCloud);
    return tempCloud;
}

template <typename T>
double stamp2Sec(const T& stamp) {
    return rclcpp::Time(stamp).seconds();
}

inline float pointDistance(PointType p) { return sqrt(p.x * p.x + p.y * p.y + p.z * p.z); }

inline float pointDistance(PointType p1, PointType p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

inline rmw_qos_profile_t qos_profile{RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                     1,
                                     RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                     RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                     RMW_QOS_DEADLINE_DEFAULT,
                                     RMW_QOS_LIFESPAN_DEFAULT,
                                     RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                     RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                     false};

inline auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

inline rmw_qos_profile_t qos_profile_imu{RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                         2000,
                                         RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                         RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                         RMW_QOS_DEADLINE_DEFAULT,
                                         RMW_QOS_LIFESPAN_DEFAULT,
                                         RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                         RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                         false};

inline auto qos_imu = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_imu.history, qos_profile_imu.depth), qos_profile_imu);

inline rmw_qos_profile_t qos_profile_lidar{RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                           5,
                                           RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                           RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                           RMW_QOS_DEADLINE_DEFAULT,
                                           RMW_QOS_LIFESPAN_DEFAULT,
                                           RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                           RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                           false};

inline auto qos_lidar = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_lidar.history, qos_profile_lidar.depth), qos_profile_lidar);

#endif
