/**
 * @file data_reader.h
 * @brief Data reader for pcap and bag files containing Livox MID360 LiDAR and IMU data
 * 
 * This module provides functionality to read sensor data from:
 * - PCAP files (captured via Livox SDK)
 * - Custom bag files (simplified rosbag format without ROS dependency)
 */

#ifndef DATA_READER_H
#define DATA_READER_H

#include <string>
#include <vector>
#include <queue>
#include <deque>
#include <memory>
#include <fstream>
#include <functional>
#include "msg.h"

// Forward declarations for PCL types
namespace pcl {
    struct PointXYZINormal;
    template<typename T> class PointCloud;
}

/**
 * @brief Livox point structure (matches Livox SDK format)
 */
struct LivoxPoint {
    float x;
    float y;
    float z;
    float reflectivity;
    uint8_t tag;
    uint8_t line;
    double offset_time;  // ns offset from scan start
};

/**
 * @brief IMU data structure
 */
struct ImuData {
    double timestamp;      // seconds
    double gyro_x, gyro_y, gyro_z;     // rad/s
    double accel_x, accel_y, accel_z;  // m/s^2
};

/**
 * @brief Point cloud scan data
 */
struct PointCloudData {
    double timestamp;      // seconds
    std::vector<LivoxPoint> points;
};

/**
 * @brief Callback types for data processing
 */
using ImuCallback = std::function<void(const sensor_msgs::Imu::ConstPtr&)>;
using PointCloudCallback = std::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>;

/**
 * @brief Abstract base class for data readers
 */
class DataReader {
public:
    virtual ~DataReader() = default;
    
    /**
     * @brief Open a data file
     * @param filepath Path to the data file
     * @return true if successful
     */
    virtual bool open(const std::string& filepath) = 0;
    
    /**
     * @brief Check if more data is available
     * @return true if more data can be read
     */
    virtual bool hasNext() = 0;
    
    /**
     * @brief Read next data packet and dispatch to callbacks
     * @return true if data was read successfully
     */
    virtual bool readNext() = 0;
    
    /**
     * @brief Close the data file
     */
    virtual void close() = 0;
    
    /**
     * @brief Set IMU data callback
     */
    void setImuCallback(ImuCallback cb) { imu_callback_ = cb; }
    
    /**
     * @brief Set PointCloud data callback  
     */
    void setPointCloudCallback(PointCloudCallback cb) { pointcloud_callback_ = cb; }
    
protected:
    ImuCallback imu_callback_;
    PointCloudCallback pointcloud_callback_;
    
    /**
     * @brief Convert ImuData to sensor_msgs::Imu and dispatch
     */
    void dispatchImu(const ImuData& imu);
    
    /**
     * @brief Convert PointCloudData to sensor_msgs::PointCloud2 and dispatch
     */
    void dispatchPointCloud(const PointCloudData& pc);
};

/**
 * @brief PCAP file reader for Livox MID360 data
 * 
 * Parses UDP packets from pcap files captured from Livox LiDAR
 */
class PcapReader : public DataReader {
public:
    PcapReader();
    ~PcapReader() override;
    
    bool open(const std::string& filepath) override;
    bool hasNext() override;
    bool readNext() override;
    void close() override;
    
private:
    std::ifstream file_;
    std::vector<uint8_t> buffer_;
    size_t file_size_;
    size_t current_pos_;
    bool is_open_;
    size_t link_header_size_;  // Size of link layer header (Ethernet=14, RawIP=0)
    
    // Accumulated points for current scan
    PointCloudData current_scan_;
    double scan_start_time_;
    double last_point_time_;
    bool frame_id_init_;
    uint8_t current_frame_id_;
    double last_time_interval_ns_;
    double last_imu_packet_time_;
    double earliest_imu_time_;  // Track earliest IMU timestamp for alignment
    const double SCAN_PERIOD = 0.1;  // 100ms = 10Hz
    
    // Buffering for IMU/LiDAR synchronization
    std::deque<ImuData> imu_buffer_;
    std::deque<PointCloudData> lidar_buffer_;
    bool first_scan_emitted_;
    static const size_t MAX_BUFFER_SIZE = 100;  // Max buffered items
    
    // Helper function to emit current scan
    void emitCurrentScan();
    
    // Parse Livox point data packet
    bool parseLivoxPacket(const uint8_t* data, size_t len);
    bool parseLivoxImuPacket(const uint8_t* data, size_t len);
    
    // PCAP file structures
    struct PcapGlobalHeader {
        uint32_t magic_number;
        uint16_t version_major;
        uint16_t version_minor;
        int32_t thiszone;
        uint32_t sigfigs;
        uint32_t snaplen;
        uint32_t network;
    };
    
    struct PcapPacketHeader {
        uint32_t ts_sec;
        uint32_t ts_usec;
        uint32_t incl_len;
        uint32_t orig_len;
    };
    
    PcapGlobalHeader global_header_;
};

/**
 * @brief Custom bag file reader (simplified rosbag-like format)
 * 
 * Reads a custom binary format that stores IMU and PointCloud data
 */
class CustomBagReader : public DataReader {
public:
    CustomBagReader();
    ~CustomBagReader() override;
    
    bool open(const std::string& filepath) override;
    bool hasNext() override;
    bool readNext() override;
    void close() override;
    
private:
    std::ifstream file_;
    bool is_open_;
    
    // Message types
    enum MessageType : uint8_t {
        MSG_IMU = 1,
        MSG_POINTCLOUD = 2
    };
    
    bool readImuMessage();
    bool readPointCloudMessage();
};

/**
 * @brief Factory function to create appropriate reader based on file extension
 */
std::unique_ptr<DataReader> createDataReader(const std::string& filepath);

#endif // DATA_READER_H
