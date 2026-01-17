#ifndef INTERFACE_H
#define INTERFACE_H

#include <optional>
#include "msg.h"

#if defined(_WIN32) || defined(__CYGWIN__)
  #ifdef _MSC_VER
    #define FASTLIO_EXPORT __declspec(dllexport)
  #else
    #define FASTLIO_EXPORT __attribute__((dllexport))
  #endif
#else
  #define FASTLIO_EXPORT __attribute__((visibility("default")))
#endif

struct LidarOutput {
    std::optional<sensor_msgs::PointCloud2> pubLaserCloudFull;
    std::optional<sensor_msgs::PointCloud2> pubLaserCloudFull_body;
    std::optional<sensor_msgs::PointCloud2> pubLaserCloudEffect;
    std::optional<sensor_msgs::PointCloud2> pubLaserCloudMap;
    std::optional<nav_msgs::Odometry> pubOdomAftMapped;
    std::optional<nav_msgs::Path> pubPath;
};

// Configuration structure for FAST-LIO
struct FastLioConfig {
    int lidar_type = 5;  // Default to MID360 (5)
    int scan_line = 4;
    double blind = 0.5;
    double acc_cov = 0.1;
    double gyr_cov = 0.1;
    double b_acc_cov = 0.0001;
    double b_gyr_cov = 0.0001;
    double fov_deg = 360;
    double det_range = 100.0;
    bool extrinsic_est_en = false;
    std::vector<double> extrinsic_T = {0.0, 0.0, 0.0};
    std::vector<double> extrinsic_R = {1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0};
    bool time_sync_en = false;
    double time_offset_lidar_to_imu = 0.0;
    int point_filter_num = 1;
    bool pcd_save_en = true;
    int pcd_save_interval = -1;
    // Additional parameters matching original FAST-LIO
    double filter_size_corner = 0.5;
    double filter_size_surf = 0.5;
    double filter_size_map = 0.5;
    double cube_side_length = 200;
    int scan_rate = 10;
    int timestamp_unit = 1;  // 0=SEC, 1=MS, 2=US, 3=NS
};

FASTLIO_EXPORT void set_config(const FastLioConfig& config);
FASTLIO_EXPORT void init();
FASTLIO_EXPORT std::optional<LidarOutput> run();
FASTLIO_EXPORT void save_map();
FASTLIO_EXPORT void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
FASTLIO_EXPORT void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);

#endif
