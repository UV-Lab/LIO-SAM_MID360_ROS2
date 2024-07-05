#include "utility.hpp"

struct smoothness_t {
    float value;
    size_t ind;
};

struct by_value {
    bool operator()(smoothness_t const &left, smoothness_t const &right) { return left.value < right.value; }
};

class FeatureExtraction : public ParamServer {
public:
    pcl::PointCloud<PointType>::Ptr extractedCloud{new pcl::PointCloud<PointType>};
    pcl::PointCloud<PointType>::Ptr cornerCloud{new pcl::PointCloud<PointType>};
    pcl::PointCloud<PointType>::Ptr surfaceCloud{new pcl::PointCloud<PointType>};

    pcl::VoxelGrid<PointType> downSizeFilter;

    CloudInfo cloudInfo;
    std_msgs::msg::Header cloudHeader;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;

    FeatureExtraction() : ParamServer("FeatureExtractionParamServer") { initializationValue(); }

    void initializationValue() {
        cloudSmoothness.resize(N_SCAN * Horizon_SCAN);
        downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

        cloudCurvature = new float[N_SCAN * Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCAN * Horizon_SCAN];
        cloudLabel = new int[N_SCAN * Horizon_SCAN];
    }

    void FeatureExtractionHandler(CloudInfo &msgIn) {
        // cloudInfo = std::move(msgIn);                                // new cloud info
        cloudInfo = msgIn;
        cloudHeader = cloudInfo.header;  // new cloud header
        extractedCloud = cloudInfo.cloud_deskewed;
        // pcl::fromROSMsg(cloudInfo.cloud_deskewed, *extractedCloud);  // new cloud for extraction

        calculateSmoothness();

        markOccludedPoints();

        extractFeatures();

        publishFeatureCloud();
    }

    void calculateSmoothness() {
        int cloudSize = extractedCloud->points.size();
        for (int i = 5; i < cloudSize - 5; i++) {
            // float diffRange = cloudInfo.point_range[i-5] + cloudInfo.point_range[i-4]
            //                 + cloudInfo.point_range[i-3] + cloudInfo.point_range[i-2]
            //                 + cloudInfo.point_range[i-1] - cloudInfo.point_range[i] * 10
            //                 + cloudInfo.point_range[i+1] + cloudInfo.point_range[i+2]
            //                 + cloudInfo.point_range[i+3] + cloudInfo.point_range[i+4]
            //                 + cloudInfo.point_range[i+5];

            float diffRange = cloudInfo.point_range[i - 2] + cloudInfo.point_range[i - 1] - cloudInfo.point_range[i] * 4 + cloudInfo.point_range[i + 1] +
                              cloudInfo.point_range[i + 2];

            cloudCurvature[i] = diffRange * diffRange;  // diffX * diffX + diffY * diffY + diffZ * diffZ;

            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;
            // cloudSmoothness for sorting
            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }

    void markOccludedPoints() {
        int cloudSize = extractedCloud->points.size();
        // mark occluded points and parallel beam points
        for (int i = 5; i < cloudSize - 6; ++i) {
            // occluded points
            float depth1 = cloudInfo.point_range[i];
            float depth2 = cloudInfo.point_range[i + 1];
            int columnDiff = std::abs(int(cloudInfo.point_col_ind[i + 1] - cloudInfo.point_col_ind[i]));
            if (columnDiff < 10) {
                // 10 pixel diff in range image
                if (depth1 - depth2 > 0.3) {
                    // cloudNeighborPicked[i - 5] = 1;
                    // cloudNeighborPicked[i - 4] = 1;
                    // cloudNeighborPicked[i - 3] = 1;
                    // cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                } else if (depth2 - depth1 > 0.3) {
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    // cloudNeighborPicked[i + 3] = 1;
                    // cloudNeighborPicked[i + 4] = 1;
                    // cloudNeighborPicked[i + 5] = 1;
                    // cloudNeighborPicked[i + 6] = 1;
                }
            }
            // parallel beam
            float diff1 = std::abs(float(cloudInfo.point_range[i - 1] - cloudInfo.point_range[i]));
            float diff2 = std::abs(float(cloudInfo.point_range[i + 1] - cloudInfo.point_range[i]));

            if (diff1 > 0.1 * cloudInfo.point_range[i] && diff2 > 0.1 * cloudInfo.point_range[i]) cloudNeighborPicked[i] = 1;
        }
    }

    void extractFeatures() {
        cornerCloud->clear();
        surfaceCloud->clear();

        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

        for (int i = 0; i < N_SCAN; i++) {
            surfaceCloudScan->clear();

            // 将一条扫描线扫描一周的点云数据，划分为6段，每段分开提取有限数量的特征，保证特征均匀分布
            for (int j = 0; j < 6; j++) {
                // 每段点云的起始、结束索引；startRingIndex为扫描线起始第5个激光点在一维数组中的索引
                int sp = (cloudInfo.start_ring_index[i] * (6 - j) + cloudInfo.end_ring_index[i] * j) / 6;
                int ep = (cloudInfo.start_ring_index[i] * (5 - j) + cloudInfo.end_ring_index[i] * (j + 1)) / 6 - 1;

                if (sp >= ep) continue;

                // 按照曲率从小到大排序点云
                std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());

                // 按照曲率从大到小遍历
                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--) {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold) {
                        largestPickedNum++;
                        if (largestPickedNum <= 40) {
                            cloudLabel[ind] = 1;
                            cornerCloud->push_back(extractedCloud->points[ind]);
                        } else {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++) {
                            int columnDiff = std::abs(int(cloudInfo.point_col_ind[ind + l] - cloudInfo.point_col_ind[ind + l - 1]));
                            if (columnDiff > 10) break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            int columnDiff = std::abs(int(cloudInfo.point_col_ind[ind + l] - cloudInfo.point_col_ind[ind + l + 1]));
                            if (columnDiff > 10) break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++) {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold) {
                        cloudLabel[ind] = -1;
                        cloudNeighborPicked[ind] = 1;

                        for (int l = 1; l <= 5; l++) {
                            int columnDiff = std::abs(int(cloudInfo.point_col_ind[ind + l] - cloudInfo.point_col_ind[ind + l - 1]));
                            if (columnDiff > 10) break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            int columnDiff = std::abs(int(cloudInfo.point_col_ind[ind + l] - cloudInfo.point_col_ind[ind + l + 1]));
                            if (columnDiff > 10) break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++) {
                    if (cloudLabel[k] <= 0) {
                        surfaceCloudScan->push_back(extractedCloud->points[k]);
                    }
                }
            }

            surfaceCloudScanDS->clear();
            downSizeFilter.setInputCloud(surfaceCloudScan);
            downSizeFilter.filter(*surfaceCloudScanDS);

            *surfaceCloud += *surfaceCloudScanDS;
        }
    }

    void freeCloudInfoMemory() {
        cloudInfo.start_ring_index.clear();
        cloudInfo.end_ring_index.clear();
        cloudInfo.point_col_ind.clear();
        cloudInfo.point_range.clear();
    }

    void publishFeatureCloud() {
        // free cloud info memory
        freeCloudInfoMemory();
        // save newly extracted features
        *cloudInfo.cloud_corner = std::move(*cornerCloud);
        *cloudInfo.cloud_surface = std::move(*surfaceCloud);
        // publish to mapOptimization
        // pubLaserCloudInfo->publish(cloudInfo);
    }
};
