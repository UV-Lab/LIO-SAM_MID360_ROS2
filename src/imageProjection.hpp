#include "utility.hpp"
#include <pcl/range_image/range_image.h>
#include <opencv2/opencv.hpp>

struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

struct LiovxPointCustomMsg {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float time;
    uint16_t ring;
    uint16_t tag;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(LiovxPointCustomMsg,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(uint16_t, ring, ring)(uint16_t, tag,
                                                                                                                                                tag))

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(uint16_t, reflectivity, reflectivity)(
                                      uint8_t, ring, ring)(uint16_t, noise, noise)(uint32_t, range, range))

// Use the Velodyne point format as a common representation
using PointXYZIRT = LiovxPointCustomMsg;

const int queueLength = 2000;

template <typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z) {
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

template <typename T>
void imuAccel2rosAccel(sensor_msgs::msg::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z) {
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}

template <typename T>
void imuRPY2rosRPY(sensor_msgs::msg::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw) {
    double imuRoll, imuPitch, imuYaw;
    tf2::Quaternion orientation;
    tf2::fromMsg(thisImuMsg->orientation, orientation);
    tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

class ImageProjection : public ParamServer {
private:
    std::mutex imuLock;
    std::mutex odoLock;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud;

    std::deque<sensor_msgs::msg::Imu> imuQueue;
    std::deque<nav_msgs::msg::Odometry> odomQueue;
    std::deque<livox_ros_driver2::msg::CustomMsg> cloudQueue;
    livox_ros_driver2::msg::CustomMsg currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<PointType>::Ptr fullCloud;
    pcl::PointCloud<PointType>::Ptr extractedCloud;

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    double timeScanCur;
    double timeScanEnd;
    std_msgs::msg::Header cloudHeader;

    vector<int> columnIdnCountVec;

public:
    CloudInfo cloudInfo;

    ImageProjection() : ParamServer(), deskewFlag(0) {
        pubExtractedCloud = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/deskew/cloud_deskewed", 1);

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory() {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN * Horizon_SCAN);

        cloudInfo.start_ring_index.assign(N_SCAN, 0);
        cloudInfo.end_ring_index.assign(N_SCAN, 0);

        cloudInfo.point_col_ind.assign(N_SCAN * Horizon_SCAN, 0);
        cloudInfo.point_range.assign(N_SCAN * Horizon_SCAN, 0);
    }

    void resetParameters() {
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i) {
            imuTime[i] = 0;
            imuRotX[i] = 0, imuRotY[i] = 0, imuRotZ[i] = 0;
        }

        columnIdnCountVec.assign(N_SCAN, 0);
    }

    ~ImageProjection() {}

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg) {
        sensor_msgs::msg::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);

        // debug IMU data
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x <<
        //       ", y: " << thisImu.linear_acceleration.y <<
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x <<
        //       ", y: " << thisImu.angular_velocity.y <<
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
    }

    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometryMsg) {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    bool cloudHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr laserCloudMsg) {
        if (!cachePointCloud(laserCloudMsg)) return false;

        if (!deskewInfo()) return false;

        projectPointCloud();

        cloudExtraction();

        publishClouds();

        resetParameters();

        return true;
    }

    void moveFromCustomMsg(livox_ros_driver2::msg::CustomMsg &Msg, pcl::PointCloud<PointXYZIRT> &cloud) {
        cloud.clear();
        cloud.reserve(Msg.point_num);
        PointXYZIRT point;

        cloud.header.frame_id = Msg.header.frame_id;
        cloud.header.stamp = (uint64_t)((Msg.header.stamp.sec * 1e9 + Msg.header.stamp.nanosec) / 1000);
        // cloud.header.seq=Msg.header.seq;

        for (uint i = 0; i < Msg.point_num - 1; i++) {
            point.x = Msg.points[i].x;
            point.y = Msg.points[i].y;
            point.z = Msg.points[i].z;
            point.intensity = Msg.points[i].reflectivity;
            point.tag = Msg.points[i].tag;
            point.time = Msg.points[i].offset_time * 1e-9;
            point.ring = Msg.points[i].line;
            cloud.push_back(point);
        }
    }

    bool cachePointCloud(const livox_ros_driver2::msg::CustomMsg::SharedPtr &laserCloudMsg) {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2) return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();

        if (sensor == SensorType::LIVOX) {
            moveFromCustomMsg(currentCloudMsg, *laserCloudIn);
        } else {
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown sensor type: " << int(sensor));
            rclcpp::shutdown();
        }

        // get timestamp
        cloudHeader = currentCloudMsg.header;
        timeScanCur = stamp2Sec(cloudHeader.stamp);
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        // check dense flag
        if (laserCloudIn->is_dense == false) {
            RCLCPP_ERROR(get_logger(), "Point cloud is not in dense format, please remove NaN points first!");
            rclcpp::shutdown();
        }

        // // check ring channel
        // static int ringFlag = 0;
        // if (ringFlag == 0)
        // {
        //     ringFlag = -1;
        //     for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
        //     {
        //         if (currentCloudMsg.fields[i].name == "ring")
        //         {
        //             ringFlag = 1;
        //             break;
        //         }
        //     }
        //     if (ringFlag == -1)
        //     {
        //         RCLCPP_ERROR(get_logger(), "Point cloud ring channel not available, please configure your point cloud data!");
        //         rclcpp::shutdown();
        //     }
        // }

        // // check point time
        // if (deskewFlag == 0)
        // {
        //     deskewFlag = -1;
        //     for (auto &field : currentCloudMsg.fields)
        //     {
        //         if (field.name == "time" || field.name == "t")
        //         {
        //             deskewFlag = 1;
        //             break;
        //         }
        //     }
        //     if (deskewFlag == -1)
        //         RCLCPP_WARN(get_logger(), "Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        // }

        return true;
    }

    bool deskewInfo() {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        if (imuQueue.empty() || stamp2Sec(imuQueue.front().header.stamp) > timeScanCur || stamp2Sec(imuQueue.back().header.stamp) < timeScanEnd) {
            RCLCPP_INFO(get_logger(), "Waiting for IMU data ...");
            return false;
        }

        imuDeskewInfo();

        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo() {
        cloudInfo.imu_available = false;

        // 丢弃太慢imu帧
        while (!imuQueue.empty()) {
            if (stamp2Sec(imuQueue.front().header.stamp) < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty()) return;

        imuPointerCur = 0;
        // 遍历所有缓存
        for (int i = 0; i < (int)imuQueue.size(); ++i) {
            sensor_msgs::msg::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = stamp2Sec(thisImuMsg.header.stamp);

            // get roll, pitch, and yaw estimation for this scan
            if (currentImuTime <= timeScanCur) imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imu_roll_init, &cloudInfo.imu_pitch_init, &cloudInfo.imu_yaw_init);
            if (currentImuTime > timeScanEnd + 0.01) break;

            if (imuPointerCur == 0) {
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            double timeDiff = currentImuTime - imuTime[imuPointerCur - 1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur - 1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur - 1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur - 1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0) return;

        cloudInfo.imu_available = true;
    }

    void odomDeskewInfo() {
        cloudInfo.odom_available = false;

        while (!odomQueue.empty()) {
            if (stamp2Sec(odomQueue.front().header.stamp) < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty()) return;

        if (stamp2Sec(odomQueue.front().header.stamp) > timeScanCur) return;

        // get start odometry at the beinning of the scan
        nav_msgs::msg::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i) {
            startOdomMsg = odomQueue[i];

            if (stamp2Sec(startOdomMsg.header.stamp) < timeScanCur)
                continue;
            else
                break;
        }

        tf2::Quaternion orientation;
        tf2::fromMsg(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        cloudInfo.initial_guess_x = startOdomMsg.pose.pose.position.x;
        cloudInfo.initial_guess_y = startOdomMsg.pose.pose.position.y;
        cloudInfo.initial_guess_z = startOdomMsg.pose.pose.position.z;
        cloudInfo.initial_guess_roll = roll;
        cloudInfo.initial_guess_pitch = pitch;
        cloudInfo.initial_guess_yaw = yaw;

        cloudInfo.odom_available = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (stamp2Sec(odomQueue.back().header.stamp) < timeScanEnd) return;

        nav_msgs::msg::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i) {
            endOdomMsg = odomQueue[i];

            if (stamp2Sec(endOdomMsg.header.stamp) < timeScanEnd)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0]))) return;

        Eigen::Affine3f transBegin =
            pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf2::fromMsg(endOdomMsg.pose.pose.orientation, orientation);
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd =
            pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur) {
        *rotXCur = 0;
        *rotYCur = 0;
        *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur) {
            if (pointTime < imuTime[imuPointerFront]) break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0) {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur) {
        *posXCur = 0;
        *posYCur = 0;
        *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    PointType deskewPoint(PointType *point, double relTime) {
        if (deskewFlag == -1 || cloudInfo.imu_available == false) return *point;

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true) {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0, 0) * point->x + transBt(0, 1) * point->y + transBt(0, 2) * point->z + transBt(0, 3);
        newPoint.y = transBt(1, 0) * point->x + transBt(1, 1) * point->y + transBt(1, 2) * point->z + transBt(1, 3);
        newPoint.z = transBt(2, 0) * point->x + transBt(2, 1) * point->y + transBt(2, 2) * point->z + transBt(2, 3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    void projectPointCloud() {
        int cloudSize = laserCloudIn->points.size();
        // range image projection
        for (int i = 0; i < cloudSize; ++i) {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange) continue;

            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN) continue;

            if (rowIdn % downsampleRate != 0) continue;

            int columnIdn = -1;
            if (sensor == SensorType::VELODYNE || sensor == SensorType::OUSTER) {
                float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                static float ang_res_x = 360.0 / float(Horizon_SCAN);
                columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
                if (columnIdn >= Horizon_SCAN) columnIdn -= Horizon_SCAN;
            } else if (sensor == SensorType::LIVOX) {
                columnIdn = columnIdnCountVec[rowIdn];
                columnIdnCountVec[rowIdn] += 1;
            }

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN) continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX) continue;

            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
    }

    void cloudExtraction() {
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i) {
            cloudInfo.start_ring_index[i] = count - 1 + 5;
            for (int j = 0; j < Horizon_SCAN; ++j) {
                if (rangeMat.at<float>(i, j) != FLT_MAX) {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.point_col_ind[count] = j;
                    // save range info
                    cloudInfo.point_range[count] = rangeMat.at<float>(i, j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.end_ring_index[i] = count - 1 - 5;
        }
    }

    void publishClouds() {
        cloudInfo.header = cloudHeader;
        *cloudInfo.cloud_deskewed = std::move(*extractedCloud);
        if (useRviz) publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
    }
};
