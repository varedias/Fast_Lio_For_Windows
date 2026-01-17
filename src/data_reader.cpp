/**
 * @file data_reader.cpp
 * @brief Implementation of data readers for pcap and bag files
 */

#include "data_reader.h"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// DataReader base class implementation
// ============================================================================

void DataReader::dispatchImu(const ImuData& imu) {
    if (!imu_callback_) return;
    
    auto msg = std::make_shared<sensor_msgs::Imu>();
    msg->header.stamp = Time::fromSec(imu.timestamp);
    msg->header.frame_id = "body";
    msg->angular_velocity.x = imu.gyro_x;
    msg->angular_velocity.y = imu.gyro_y;
    msg->angular_velocity.z = imu.gyro_z;
    msg->linear_acceleration.x = imu.accel_x;
    msg->linear_acceleration.y = imu.accel_y;
    msg->linear_acceleration.z = imu.accel_z;
    
    // Debug IMU timestamp
    static int imu_debug = 0;
    static double first_imu_time = 0;
    if (imu_debug == 0) {
        first_imu_time = imu.timestamp;
    }
    if (imu_debug < 5) {
        std::cout << "[dispatchIMU] IMU #" << imu_debug << " timestamp=" << imu.timestamp 
                  << " (rel=" << (imu.timestamp - first_imu_time) << "s)"
                  << " acc=(" << imu.accel_x << "," << imu.accel_y << "," << imu.accel_z << ")"
                  << " gyro=(" << imu.gyro_x << "," << imu.gyro_y << "," << imu.gyro_z << ")" << std::endl;
        imu_debug++;
    }
    
    imu_callback_(msg);
}

void DataReader::dispatchPointCloud(const PointCloudData& pc) {
    if (!pointcloud_callback_) return;
    
    // Find min and max offset_time to normalize timestamps
    double min_time_ns = std::numeric_limits<double>::max();
    double max_time_ns = std::numeric_limits<double>::lowest();
    for (const auto& pt : pc.points) {
        if (pt.offset_time < min_time_ns) min_time_ns = pt.offset_time;
        if (pt.offset_time > max_time_ns) max_time_ns = pt.offset_time;
    }
    
    // Debug: print first few points of first few scans
    static int scan_debug = 0;
    static double first_lidar_time = 0;
    if (scan_debug == 0) {
        first_lidar_time = pc.timestamp;
    }
    if (scan_debug < 3 && pc.points.size() > 5) {
        std::cout << "[dispatchPointCloud] Scan #" << scan_debug 
                  << " timestamp=" << pc.timestamp 
                  << " (rel=" << (pc.timestamp - first_lidar_time) << "s)"
                  << " normalization: min_time_ns=" << min_time_ns 
                  << " max_time_ns=" << max_time_ns << " span_ms=" << ((max_time_ns - min_time_ns) * 1e-6) << std::endl;
        std::cout << "[dispatchPointCloud] Scan #" << scan_debug << " raw points (in meters):" << std::endl;
        for (size_t i = 0; i < 5; i++) {
            float normalized_time_ms = (pc.points[i].offset_time - min_time_ns) * 1e-6;
            std::cout << "  [" << i << "] x=" << pc.points[i].x 
                      << " y=" << pc.points[i].y 
                      << " z=" << pc.points[i].z 
                      << " time_ms=" << normalized_time_ms << std::endl;
        }
        scan_debug++;
    }
    
    auto msg = std::make_shared<sensor_msgs::PointCloud2>();
    msg->header.stamp = Time::fromSec(pc.timestamp);
    msg->header.frame_id = "lidar";
    
    // Set up fields for XYZI + time + ring
    msg->fields = {
        {"x", 0, 7, 1},           // FLOAT32
        {"y", 4, 7, 1},           // FLOAT32
        {"z", 8, 7, 1},           // FLOAT32
        {"intensity", 12, 7, 1},  // FLOAT32
        {"time", 16, 7, 1},       // FLOAT32 (curvature field used for time)
        {"ring", 20, 2, 1}        // UINT8
    };
    
    msg->point_step = 24;  // bytes per point
    msg->height = 1;
    msg->width = static_cast<uint32_t>(pc.points.size());
    msg->row_step = msg->point_step * msg->width;
    msg->is_dense = true;
    msg->is_bigendian = false;
    
    // Pack point data with normalized time (starting from 0)
    msg->data.resize(msg->row_step);
    uint8_t* ptr = msg->data.data();
    
    for (const auto& pt : pc.points) {
        memcpy(ptr + 0, &pt.x, 4);
        memcpy(ptr + 4, &pt.y, 4);
        memcpy(ptr + 8, &pt.z, 4);
        memcpy(ptr + 12, &pt.reflectivity, 4);
        // Normalize time to start from 0, convert ns to ms (FAST-LIO expects ms in curvature field)
        float time_offset = static_cast<float>((pt.offset_time - min_time_ns) * 1e-6);
        memcpy(ptr + 16, &time_offset, 4);
        memcpy(ptr + 20, &pt.line, 1);
        ptr += msg->point_step;
    }
    
    pointcloud_callback_(msg);
}

// ============================================================================
// PcapReader implementation
// ============================================================================

PcapReader::PcapReader() 
        : file_size_(0), current_pos_(0), is_open_(false), link_header_size_(0),
            scan_start_time_(0), last_point_time_(0), frame_id_init_(false), current_frame_id_(0),
            last_time_interval_ns_(0.0), last_imu_packet_time_(0.0), earliest_imu_time_(0.0),
            first_scan_emitted_(false) {
    buffer_.resize(65536);  // Max UDP packet size
}

PcapReader::~PcapReader() {
    close();
}

void PcapReader::emitCurrentScan() {
    if (current_scan_.points.empty()) return;
    
    // Sort points by absolute timestamp (stored in offset_time as absolute ns from epoch)
    std::sort(current_scan_.points.begin(), current_scan_.points.end(),
        [](const LivoxPoint& a, const LivoxPoint& b) {
            return a.offset_time < b.offset_time;
        });
    
    // Use the earliest point's time as the frame base time
    double frame_base_time_ns = current_scan_.points.front().offset_time;
    double frame_base_time_sec = frame_base_time_ns * 1e-9;
    double frame_end_time_ns = frame_base_time_ns + SCAN_PERIOD * 1e9;  // 100ms in ns
    
    // Skip frames that start before we have IMU coverage
    if (!first_scan_emitted_ && earliest_imu_time_ > 0 && frame_base_time_sec < earliest_imu_time_) {
        double time_gap = earliest_imu_time_ - frame_base_time_sec;
        static int skip_debug = 0;
        if (skip_debug < 5) {
            std::cout << "[PcapReader] Skipping frame: timestamp=" << frame_base_time_sec
                      << " earliest_imu=" << earliest_imu_time_
                      << " gap=" << time_gap << "s - need IMU coverage" << std::endl;
            skip_debug++;
        }
        current_scan_.points.clear();
        scan_start_time_ = 0;
        return;
    }
    
    // Find the split point: first point that belongs to next frame (time >= frame_end)
    auto split_it = std::partition_point(current_scan_.points.begin(), current_scan_.points.end(),
        [frame_end_time_ns](const LivoxPoint& pt) {
            return pt.offset_time < frame_end_time_ns;
        });
    
    // If all points belong to current frame, emit them all
    // Otherwise, save the overflow points for the next frame
    std::vector<LivoxPoint> overflow_points;
    if (split_it != current_scan_.points.end()) {
        overflow_points.assign(split_it, current_scan_.points.end());
        current_scan_.points.erase(split_it, current_scan_.points.end());
    }
    
    // Now recalculate offset_time to be relative to frame start (still in ns)
    for (auto& pt : current_scan_.points) {
        pt.offset_time = pt.offset_time - frame_base_time_ns;
    }
    
    std::cout << "[PcapReader] Emitting scan with " << current_scan_.points.size() << " points" << std::endl;
    
    // Use the sorted first point's time as frame timestamp
    current_scan_.timestamp = frame_base_time_sec;
    
    static int emit_debug = 0;
    if (emit_debug < 5) {
        double span_ms = current_scan_.points.empty() ? 0 : (current_scan_.points.back().offset_time) * 1e-6;
        std::cout << "[PcapReader] Frame timestamp=" << current_scan_.timestamp 
                  << " earliest_imu=" << earliest_imu_time_ 
                  << " first_scan_emitted=" << first_scan_emitted_ 
                  << " span_ms=" << span_ms 
                  << " overflow=" << overflow_points.size() << std::endl;
        emit_debug++;
    }
    
    if (!current_scan_.points.empty()) {
        dispatchPointCloud(current_scan_);
        first_scan_emitted_ = true;
    }
    
    // Reset for next scan and add overflow points
    current_scan_.points.clear();
    current_scan_.points = std::move(overflow_points);
    
    // Set scan_start_time_ based on overflow points, or 0 if none
    if (!current_scan_.points.empty()) {
        // Use the first overflow point's time as the new scan_start_time
        scan_start_time_ = current_scan_.points.front().offset_time * 1e-9;
    } else {
        scan_start_time_ = 0;
    }
}

bool PcapReader::open(const std::string& filepath) {
    file_.open(filepath, std::ios::binary);
    if (!file_.is_open()) {
        std::cerr << "[PcapReader] Failed to open: " << filepath << std::endl;
        return false;
    }
    
    // Get file size
    file_.seekg(0, std::ios::end);
    file_size_ = file_.tellg();
    file_.seekg(0, std::ios::beg);
    
    // Read global header
    file_.read(reinterpret_cast<char*>(&global_header_), sizeof(global_header_));
    
    // Validate magic number
    if (global_header_.magic_number != 0xa1b2c3d4 && 
        global_header_.magic_number != 0xd4c3b2a1) {
        std::cerr << "[PcapReader] Invalid PCAP magic number" << std::endl;
        file_.close();
        return false;
    }
    
    current_pos_ = file_.tellg();
    is_open_ = true;
    
    // Determine link layer header size based on network type
    // 1 = Ethernet, 101 = Raw IP, 113 = Linux cooked capture
    switch (global_header_.network) {
        case 1:   link_header_size_ = 14; break;  // Ethernet
        case 101: link_header_size_ = 0;  break;  // Raw IP
        case 113: link_header_size_ = 16; break;  // Linux cooked
        default:  link_header_size_ = 0;  break;
    }
    
    std::cout << "[PcapReader] Opened: " << filepath << std::endl;
    std::cout << "[PcapReader] File size: " << (file_size_ / 1024.0 / 1024.0) << " MB" << std::endl;
    std::cout << "[PcapReader] Network type: " << global_header_.network 
              << " (link header: " << link_header_size_ << " bytes)" << std::endl;
    
    return true;
}

bool PcapReader::hasNext() {
    return is_open_ && current_pos_ < file_size_ - sizeof(PcapPacketHeader);
}

bool PcapReader::readNext() {
    if (!hasNext()) return false;
    
    PcapPacketHeader packet_header;
    file_.read(reinterpret_cast<char*>(&packet_header), sizeof(packet_header));
    
    if (file_.gcount() != sizeof(packet_header)) {
        return false;
    }
    
    // Read packet data
    if (packet_header.incl_len > buffer_.size()) {
        buffer_.resize(packet_header.incl_len);
    }
    
    file_.read(reinterpret_cast<char*>(buffer_.data()), packet_header.incl_len);
    
    if (static_cast<uint32_t>(file_.gcount()) != packet_header.incl_len) {
        return false;
    }
    
    current_pos_ = file_.tellg();
    
    // Calculate header offsets based on link layer type
    // IP header: 20 bytes (minimum)
    // UDP header: 8 bytes
    size_t ip_offset = link_header_size_;
    size_t udp_offset = ip_offset + 20;
    size_t payload_offset = udp_offset + 8;
    
    if (packet_header.incl_len < payload_offset) {
        return true;  // Skip packets too small
    }
    
    // Debug: show first few packets
    static int debug_count = 0;
    if (false && debug_count < 10) {  // Disabled verbose debug
        uint8_t ip_protocol = buffer_[ip_offset + 9];
        uint16_t dst_port = (buffer_[udp_offset + 2] << 8) | buffer_[udp_offset + 3];
        uint16_t src_port = (buffer_[udp_offset + 0] << 8) | buffer_[udp_offset + 1];
        
        std::cout << "[PcapReader] Pkt " << debug_count 
                  << ": proto=" << (int)ip_protocol 
                  << " src_port=" << src_port 
                  << " dst_port=" << dst_port 
                  << " len=" << packet_header.incl_len << std::endl;
        debug_count++;
    }
    
    // Check if it's a UDP packet (IP protocol = 17, at offset 9 in IP header)
    uint8_t ip_protocol = buffer_[ip_offset + 9];
    if (ip_protocol != 17) {
        return true;  // Not UDP, skip
    }
    
    // Get UDP destination port (at offset 2 in UDP header)
    uint16_t dst_port = (buffer_[udp_offset + 2] << 8) | buffer_[udp_offset + 3];
    uint16_t src_port = (buffer_[udp_offset + 0] << 8) | buffer_[udp_offset + 1];
    
    // Livox MID360 uses various ports depending on configuration
    // Common patterns:
    // - Point cloud: 56001, 56301, etc.
    // - IMU: 56101, 56401, etc.
    // The actual port depends on LiDAR configuration
    const uint8_t* payload = buffer_.data() + payload_offset;
    size_t payload_len = packet_header.incl_len - payload_offset;
    
    double timestamp = packet_header.ts_sec + packet_header.ts_usec * 1e-6;
    
    // Check for Livox ports (broader range 56000-57000)
    // Also check by source port since Livox sends data from specific ports
    bool is_livox_port = (dst_port >= 56000 && dst_port < 57000) || 
                         (src_port >= 56000 && src_port < 57000);
    
    if (is_livox_port) {
        // Livox MID360 uses LivoxLidarEthernetPacket format (SDK2)
        // Packet header structure (36 bytes):
        //   uint8_t  version;           // offset 0
        //   uint16_t length;            // offset 1
        //   uint16_t time_interval;     // offset 3, unit: 0.1us
        //   uint16_t dot_num;           // offset 5, number of points
        //   uint16_t udp_cnt;           // offset 7
        //   uint8_t  frame_cnt;         // offset 9
        //   uint8_t  data_type;         // offset 10
        //   uint8_t  time_type;         // offset 11
        //   uint8_t  rsvd[12];          // offset 12
        //   uint32_t crc32;             // offset 24
        //   uint8_t  timestamp[8];      // offset 28
        //   uint8_t  data[];            // offset 36, point cloud data
        
        const size_t LIVOX_HEADER_SIZE = 36;
        
        if (payload_len >= LIVOX_HEADER_SIZE) {
            uint8_t version = payload[0];
            uint16_t time_interval = payload[3] | (payload[4] << 8); // unit: 0.1us
            uint16_t dot_num = payload[5] | (payload[6] << 8);
            uint8_t frame_cnt = payload[9];
            uint8_t data_type = payload[10];
            
            // Get Livox internal timestamp (bytes 28-35) - nanoseconds since device power on
            uint64_t timestamp_ns;
            memcpy(&timestamp_ns, payload + 28, 8);
            
            // Use Livox internal timestamp (in seconds) for all data synchronization
            // This ensures LiDAR and IMU timestamps are consistent
            double livox_timestamp = timestamp_ns * 1e-9;  // Convert ns to seconds
            double time_interval_ns = static_cast<double>(time_interval) * 100.0; // 0.1us -> 100ns
            if (time_interval_ns > 0.0) {
                last_time_interval_ns_ = time_interval_ns;
            } else if (last_time_interval_ns_ > 0.0) {
                time_interval_ns = last_time_interval_ns_;
            }

            const bool is_lidar_packet = (data_type == 0x01 || data_type == 0x02 || data_type == 0x03);
            
            static int livox_debug_count = 0;
            if (false && livox_debug_count < 5) {  // Disabled verbose debug
                std::cout << "[PcapReader] Livox packet: version=" << (int)version 
                          << " dot_num=" << dot_num 
                          << " data_type=" << (int)data_type 
                          << " payload_len=" << payload_len 
                          << " timestamp_ns=" << timestamp_ns << std::endl;
                
                // Print first 80 bytes of payload in hex
                std::cout << "[PcapReader] Payload hex dump:" << std::endl;
                for (size_t row = 0; row < 6; row++) {
                    std::cout << "  [" << std::setw(2) << (row * 16) << "] ";
                    for (size_t i = 0; i < 16 && (row * 16 + i) < payload_len; i++) {
                        printf("%02x ", payload[row * 16 + i]);
                    }
                    std::cout << std::endl;
                }
                
                // Look for first non-zero point
                const uint8_t* point_start = payload + LIVOX_HEADER_SIZE;
                size_t point_data_len_dbg = payload_len - LIVOX_HEADER_SIZE;
                std::cout << "[PcapReader] Point data starts at offset " << LIVOX_HEADER_SIZE 
                          << ", data len=" << point_data_len_dbg << std::endl;
                
                // Check first few points
                const size_t POINT_SIZE = 14;
                for (size_t pi = 0; pi < std::min((size_t)5, point_data_len_dbg / POINT_SIZE); pi++) {
                    const uint8_t* pt = point_start + pi * POINT_SIZE;
                    int32_t x, y, z;
                    memcpy(&x, pt, 4);
                    memcpy(&y, pt + 4, 4);
                    memcpy(&z, pt + 8, 4);
                    std::cout << "  Point[" << pi << "]: x=" << x << " y=" << y << " z=" << z 
                              << " ref=" << (int)pt[12] << " tag=" << (int)pt[13] << std::endl;
                }
                
                livox_debug_count++;
            }
            
            const uint8_t* point_data = payload + LIVOX_HEADER_SIZE;
            size_t point_data_len = payload_len - LIVOX_HEADER_SIZE;
            size_t packet_point_size = 0;
            if (data_type == 0x01) packet_point_size = 14;
            else if (data_type == 0x02) packet_point_size = 8;
            else if (data_type == 0x03) packet_point_size = 10;
            size_t packet_point_count = (packet_point_size > 0)
                ? std::min(static_cast<size_t>(dot_num), point_data_len / packet_point_size)
                : 0;
            
            // IMPORTANT: According to Livox SDK2, the timestamp field in packet header is 
            // the timestamp of the FIRST point in the packet, not the last!
            // So packet_start_time = livox_timestamp directly
            double packet_start_time = livox_timestamp;
            // The time of each point is: packet_start_time + i * time_interval_ns

            // Time-based scan segmentation (ignore frame_cnt, use time only)
            // This ensures consistent 100ms frames regardless of Livox frame_cnt behavior
            if (is_lidar_packet) {
                if (scan_start_time_ == 0) {
                    scan_start_time_ = packet_start_time;
                }
                // Note: Frame segmentation is now done entirely in readNext() based on time
            }
            
            // Static counters for data type statistics
            static size_t imu_packets = 0;
            static size_t lidar_packets = 0;
            
            // Determine point size based on data_type
            // 0x00: IMU data (24 bytes per sample)
            // 0x01: Cartesian high precision (14 bytes: 3*int32 + uint8 + uint8)
            // 0x02: Cartesian low precision (8 bytes: 3*int16 + uint8 + uint8)
            // 0x03: Spherical (10 bytes: uint32 + uint16 + uint16 + uint8 + uint8)
            
            if (data_type == 0x00) {
                // IMU data - MID360 uses data_type 0x00 for IMU packets
                imu_packets++;
                
                // Debug: print IMU packet timing
                static int imu_pkt_debug = 0;
                static double first_imu_ts = 0;
                if (imu_pkt_debug == 0) first_imu_ts = livox_timestamp;
                if (imu_pkt_debug < 10) {
                    std::cout << "[PcapReader] IMU packet #" << imu_pkt_debug 
                              << ": livox_timestamp=" << livox_timestamp 
                              << " (rel=" << (livox_timestamp - first_imu_ts) << ")"
                              << " scan_start=" << scan_start_time_ << std::endl;
                    imu_pkt_debug++;
                }
                
                // Livox SDK2 IMU format: LivoxLidarImuRawPoint
                // gyro_x, gyro_y, gyro_z (float * 3 = 12 bytes) - rad/s
                // acc_x, acc_y, acc_z (float * 3 = 12 bytes) - g (need to convert to m/s²)
                // Total = 24 bytes per sample
                const size_t IMU_SIZE = 24;
                const float GRAVITY = 9.81f;  // m/s²
                size_t num_samples = point_data_len / IMU_SIZE;

                // IMPORTANT: According to Livox SDK2, the timestamp field in packet header is 
                // the timestamp of the FIRST sample in the packet, not the last!
                double imu_packet_start_time = livox_timestamp;
                // The time of each sample is: imu_packet_start_time + i * time_interval_ns
                
                for (size_t i = 0; i < num_samples; i++) {
                    const uint8_t* imu_ptr = point_data + i * IMU_SIZE;
                    
                    float gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z;
                    memcpy(&gyro_x, imu_ptr + 0, 4);
                    memcpy(&gyro_y, imu_ptr + 4, 4);
                    memcpy(&gyro_z, imu_ptr + 8, 4);
                    memcpy(&acc_x, imu_ptr + 12, 4);
                    memcpy(&acc_y, imu_ptr + 16, 4);
                    memcpy(&acc_z, imu_ptr + 20, 4);
                    
                    ImuData imu;
                    double imu_time = imu_packet_start_time + (time_interval_ns * static_cast<double>(i)) * 1e-9;
                    imu.timestamp = imu_time;
                    
                    // Track earliest IMU time for LiDAR frame alignment
                    if (earliest_imu_time_ == 0.0 || imu_time < earliest_imu_time_) {
                        earliest_imu_time_ = imu_time;
                    }
                    
                    imu.gyro_x = gyro_x;   // rad/s
                    imu.gyro_y = gyro_y;
                    imu.gyro_z = gyro_z;
                    // Keep acceleration in g units - FAST-LIO expects this!
                    // FAST-LIO will convert to m/s² internally using: acc * G_m_s2 / mean_acc.norm()
                    imu.accel_x = acc_x;   // g units (gravity units)
                    imu.accel_y = acc_y;
                    imu.accel_z = acc_z;
                    
                    dispatchImu(imu);
                }

                last_imu_packet_time_ = livox_timestamp;
            }
            else if (data_type == 0x01) {
                lidar_packets++;
                // Cartesian high precision: 14 bytes per point
                // int32_t x (mm), int32_t y (mm), int32_t z (mm), uint8_t reflectivity, uint8_t tag
                const size_t POINT_SIZE = 14;
                size_t num_points = std::min(static_cast<size_t>(dot_num), point_data_len / POINT_SIZE);
                
                static bool first_point_debug = true;
                static size_t total_points_added = 0;
                static size_t total_points_skipped = 0;
                
                if (first_point_debug && num_points > 0) {
                    int32_t x, y, z;
                    memcpy(&x, point_data, 4);
                    memcpy(&y, point_data + 4, 4);
                    memcpy(&z, point_data + 8, 4);
                    std::cout << "[PcapReader] High-precision point: x=" << x << "mm y=" << y << "mm z=" << z << "mm"
                              << " (=" << (x*0.001f) << "m, " << (y*0.001f) << "m, " << (z*0.001f) << "m)" << std::endl;
                    first_point_debug = false;
                }
                
                for (size_t i = 0; i < num_points; i++) {
                    const uint8_t* pt = point_data + i * POINT_SIZE;
                    
                    int32_t x_mm, y_mm, z_mm;
                    memcpy(&x_mm, pt + 0, 4);
                    memcpy(&y_mm, pt + 4, 4);
                    memcpy(&z_mm, pt + 8, 4);
                    
                    // Calculate this point's absolute timestamp (in seconds)
                    double point_time_sec = packet_start_time + time_interval_ns * static_cast<double>(i) * 1e-9;
                    // Store as nanoseconds from epoch for sorting later
                    double point_time_ns = point_time_sec * 1e9;
                    
                    // For first frame, skip points before IMU coverage
                    if (!first_scan_emitted_ && earliest_imu_time_ > 0 && point_time_sec < earliest_imu_time_) {
                        total_points_skipped++;
                        continue;
                    }
                    
                    // Skip invalid points (all zeros or very close)
                    float dist_sq = (x_mm * 0.001f) * (x_mm * 0.001f) + 
                                   (y_mm * 0.001f) * (y_mm * 0.001f) + 
                                   (z_mm * 0.001f) * (z_mm * 0.001f);
                    if (dist_sq < 0.01f) {  // Skip points closer than 0.1m
                        total_points_skipped++;
                        continue;
                    }
                    
                    // Set frame_start_time_ if this is the first valid point of a new frame
                    if (scan_start_time_ == 0.0) {
                        scan_start_time_ = point_time_sec;
                    }
                    
                    // Check if this point is far enough from frame start to trigger new frame
                    if (point_time_sec >= scan_start_time_ + SCAN_PERIOD) {
                        if (current_scan_.points.size() > 100) {
                            emitCurrentScan();
                            // scan_start_time_ is reset to 0 in emitCurrentScan
                            // Set it to current point's time for the new frame
                            scan_start_time_ = point_time_sec;
                        }
                    }
                    
                    LivoxPoint point;
                    point.x = x_mm * 0.001f;  // mm to m
                    point.y = y_mm * 0.001f;
                    point.z = z_mm * 0.001f;
                    point.reflectivity = static_cast<float>(pt[12]);
                    point.tag = pt[13];
                    point.line = pt[13] & 0x03;
                    // Store absolute timestamp in ns (will be converted to relative time in emitCurrentScan)
                    point.offset_time = point_time_ns;
                    
                    current_scan_.points.push_back(point);
                    total_points_added++;
                }
                
                // Print stats every 1000 packets
                static size_t pkt_count = 0;
                pkt_count++;
                if (pkt_count % 1000 == 0) {
                    std::cout << "[PcapReader] Stats: pkt=" << pkt_count 
                              << " points_added=" << total_points_added 
                              << " points_skipped=" << total_points_skipped 
                              << " current_scan=" << current_scan_.points.size() << std::endl;
                }
            }
            else if (data_type == 0x02) {
                // Cartesian low precision: 8 bytes per point
                // int16_t x (cm), int16_t y (cm), int16_t z (cm), uint8_t reflectivity, uint8_t tag
                const size_t POINT_SIZE = 8;
                size_t num_points = std::min(static_cast<size_t>(dot_num), point_data_len / POINT_SIZE);
                
                static bool first_low_debug = true;
                if (first_low_debug && num_points > 0) {
                    int16_t x, y, z;
                    memcpy(&x, point_data, 2);
                    memcpy(&y, point_data + 2, 2);
                    memcpy(&z, point_data + 4, 2);
                    std::cout << "[PcapReader] Low-precision point: x=" << x << "cm y=" << y << "cm z=" << z << "cm"
                              << " (=" << (x*0.01f) << "m, " << (y*0.01f) << "m, " << (z*0.01f) << "m)" << std::endl;
                    first_low_debug = false;
                }
                
                for (size_t i = 0; i < num_points; i++) {
                    const uint8_t* pt = point_data + i * POINT_SIZE;
                    
                    int16_t x_cm, y_cm, z_cm;
                    memcpy(&x_cm, pt + 0, 2);
                    memcpy(&y_cm, pt + 2, 2);
                    memcpy(&z_cm, pt + 4, 2);
                    
                    // Calculate this point's absolute timestamp (in seconds)
                    double point_time_sec = packet_start_time + time_interval_ns * static_cast<double>(i) * 1e-9;
                    double point_time_ns = point_time_sec * 1e9;
                    
                    // For first frame, skip points before IMU coverage
                    if (!first_scan_emitted_ && earliest_imu_time_ > 0 && point_time_sec < earliest_imu_time_) {
                        continue;
                    }
                    
                    // Skip invalid points
                    float dist_sq = (x_cm * 0.01f) * (x_cm * 0.01f) + 
                                   (y_cm * 0.01f) * (y_cm * 0.01f) + 
                                   (z_cm * 0.01f) * (z_cm * 0.01f);
                    if (dist_sq < 0.01f) {
                        continue;
                    }
                    
                    // Set frame_start_time_ if this is the first valid point of a new frame
                    if (scan_start_time_ == 0.0) {
                        scan_start_time_ = point_time_sec;
                    }
                    
                    // Check if this point triggers new frame
                    if (point_time_sec >= scan_start_time_ + SCAN_PERIOD) {
                        if (current_scan_.points.size() > 100) {
                            emitCurrentScan();
                            scan_start_time_ = point_time_sec;
                        }
                    }
                    
                    LivoxPoint point;
                    point.x = x_cm * 0.01f;  // cm to m
                    point.y = y_cm * 0.01f;
                    point.z = z_cm * 0.01f;
                    point.reflectivity = static_cast<float>(pt[6]);
                    point.tag = pt[7];
                    point.line = pt[7] & 0x03;
                    // Store absolute timestamp in ns
                    point.offset_time = point_time_ns;
                    
                    current_scan_.points.push_back(point);
                }
            }
            else if (data_type == 0x03) {
                // Spherical: 10 bytes per point
                // uint32_t depth (mm), uint16_t theta, uint16_t phi, uint8_t reflectivity, uint8_t tag
                const size_t POINT_SIZE = 10;
                size_t num_points = std::min(static_cast<size_t>(dot_num), point_data_len / POINT_SIZE);
                
                for (size_t i = 0; i < num_points; i++) {
                    const uint8_t* pt = point_data + i * POINT_SIZE;
                    
                    uint32_t depth;
                    uint16_t theta, phi;
                    memcpy(&depth, pt + 0, 4);
                    memcpy(&theta, pt + 4, 2);
                    memcpy(&phi, pt + 6, 2);
                    
                    // Calculate this point's absolute timestamp (in seconds)
                    double point_time_sec = packet_start_time + time_interval_ns * static_cast<double>(i) * 1e-9;
                    double point_time_ns = point_time_sec * 1e9;
                    
                    // For first frame, skip points before IMU coverage
                    if (!first_scan_emitted_ && earliest_imu_time_ > 0 && point_time_sec < earliest_imu_time_) {
                        continue;
                    }
                    
                    // Skip invalid points (depth check first)
                    if (depth < 100) {  // Skip points closer than 0.1m
                        continue;
                    }
                    
                    // Set frame_start_time_ if this is the first valid point of a new frame
                    if (scan_start_time_ == 0.0) {
                        scan_start_time_ = point_time_sec;
                    }
                    
                    // Check if this point triggers new frame
                    if (point_time_sec >= scan_start_time_ + SCAN_PERIOD) {
                        if (current_scan_.points.size() > 100) {
                            emitCurrentScan();
                            scan_start_time_ = point_time_sec;
                        }
                    }
                    
                    // Convert spherical to Cartesian
                    // depth in mm, theta/phi in 0.01 degree
                    float r = depth * 0.001f;  // mm to m
                    float theta_rad = theta * 0.01f * M_PI / 180.0f;
                    float phi_rad = phi * 0.01f * M_PI / 180.0f;
                    
                    LivoxPoint point;
                    point.x = r * sinf(theta_rad) * cosf(phi_rad);
                    point.y = r * sinf(theta_rad) * sinf(phi_rad);
                    point.z = r * cosf(theta_rad);
                    point.reflectivity = static_cast<float>(pt[8]);
                    point.tag = pt[9];
                    point.line = pt[9] & 0x03;
                    // Store absolute timestamp in ns
                    point.offset_time = point_time_ns;
                    
                    current_scan_.points.push_back(point);
                }
            }
            
            // Update last known timestamp (only for LiDAR packets)
            if (is_lidar_packet && packet_point_count > 0) {
                last_point_time_ = packet_start_time + time_interval_ns * (packet_point_count - 1) * 1e-9;
            }
        }
    }
    
    // Note: Frame emission is now handled at the point level within each data_type handler.
    // The emitCurrentScan() function is called when a point's offset exceeds SCAN_PERIOD.
    // This section only handles cleanup of any remaining points when EOF is reached.
    
    return true;
}

bool PcapReader::parseLivoxPacket(const uint8_t* data, size_t len) {
    // Livox HAP/MID360 Point Cloud format
    // The data after Livox protocol header varies by device
    // 
    // For MID360 (Livox SDK2):
    // Header (24 bytes):
    //   - Version (1 byte)
    //   - Slot ID (1 byte)
    //   - LiDAR ID (1 byte)
    //   - Reserved (1 byte)
    //   - Status code (4 bytes)
    //   - Timestamp type (1 byte)
    //   - Data type (1 byte): 1=Cartesian, 2=Spherical
    //   - Timestamp (8 bytes)
    //   - Reserved (6 bytes)
    //
    // Point data (varies by data_type):
    //   Cartesian: x,y,z (int32, mm), reflectivity (uint8), tag (uint8) = 14 bytes
    //   OR: x,y,z (int32, mm), reflectivity (uint8), tag (uint8), x2,y2,z2, ref2, tag2... (dual return)
    
    if (len < 24) return false;
    
    static int packet_count = 0;
    static bool first_debug = true;
    
    // Parse header
    uint8_t version = data[0];
    uint8_t timestamp_type = data[8];
    uint8_t data_type = data[9];
    
    // Get timestamp from header (bytes 10-17)
    uint64_t timestamp_ns;
    memcpy(&timestamp_ns, data + 10, 8);
    
    double timestamp_s = timestamp_ns * 1e-9;
    
    // Debug first few packets
    if (first_debug && packet_count < 5) {
        std::cout << "[PcapReader] Packet " << packet_count << ": version=" << (int)version 
                  << " ts_type=" << (int)timestamp_type << " data_type=" << (int)data_type 
                  << " len=" << len << " timestamp_ns=" << timestamp_ns << std::endl;
        packet_count++;
        if (packet_count >= 5) first_debug = false;
    }
    
    if (current_scan_.points.empty()) {
        scan_start_time_ = timestamp_s;
    }
    
    const uint8_t* point_data = data + 24;
    size_t point_data_len = len - 24;
    
    // Determine point size based on data type
    // Type 1: Cartesian single return (14 bytes)
    // Type 2: Cartesian dual return (28 bytes for 2 points)
    // For spherical types, different sizes apply
    size_t POINT_SIZE = 14;  // Default for Cartesian single return
    size_t num_points = point_data_len / POINT_SIZE;
    
    for (size_t i = 0; i < num_points; i++) {
        const uint8_t* pt = point_data + i * POINT_SIZE;
        
        int32_t x_mm, y_mm, z_mm;
        memcpy(&x_mm, pt + 0, 4);
        memcpy(&y_mm, pt + 4, 4);
        memcpy(&z_mm, pt + 8, 4);
        
        LivoxPoint point;
        point.x = x_mm * 0.001f;  // mm to m
        point.y = y_mm * 0.001f;
        point.z = z_mm * 0.001f;
        point.reflectivity = static_cast<float>(pt[12]);
        point.tag = pt[13];
        point.line = pt[13] & 0x03;  // Lower 2 bits for line number
        
        // Calculate offset time
        uint32_t offset_ns;
        memcpy(&offset_ns, pt + 10, 4);
        point.offset_time = static_cast<double>(offset_ns);
        
        // Skip invalid points
        if (point.x == 0 && point.y == 0 && point.z == 0) {
            continue;
        }
        
        current_scan_.points.push_back(point);
    }
    
    return true;
}

bool PcapReader::parseLivoxImuPacket(const uint8_t* data, size_t len) {
    // Livox IMU format (after protocol header)
    // gyro_x, gyro_y, gyro_z: float32 (rad/s)
    // acc_x, acc_y, acc_z: float32 (g)
    
    if (len < 48) return false;
    
    // Get timestamp
    uint64_t timestamp_ns;
    memcpy(&timestamp_ns, data + 8, 8);
    
    const uint8_t* imu_data = data + 24;
    
    ImuData imu;
    imu.timestamp = timestamp_ns * 1e-9;
    
    float gyro[3], accel[3];
    memcpy(gyro, imu_data, 12);
    memcpy(accel, imu_data + 12, 12);
    
    imu.gyro_x = gyro[0];
    imu.gyro_y = gyro[1];
    imu.gyro_z = gyro[2];
    imu.accel_x = accel[0] * 9.80665;  // g to m/s^2
    imu.accel_y = accel[1] * 9.80665;
    imu.accel_z = accel[2] * 9.80665;
    
    dispatchImu(imu);
    return true;
}

void PcapReader::close() {
    if (is_open_) {
        // Emit any remaining points in current scan
        if (!current_scan_.points.empty()) {
            emitCurrentScan();
        }
        file_.close();
        is_open_ = false;
    }
}

// ============================================================================
// CustomBagReader implementation
// ============================================================================

CustomBagReader::CustomBagReader() : is_open_(false) {}

CustomBagReader::~CustomBagReader() {
    close();
}

bool CustomBagReader::open(const std::string& filepath) {
    file_.open(filepath, std::ios::binary);
    if (!file_.is_open()) {
        std::cerr << "[CustomBagReader] Failed to open: " << filepath << std::endl;
        return false;
    }
    
    // Read and validate header
    char magic[8];
    file_.read(magic, 8);
    
    // Check if this is our custom format or try to handle as raw data
    is_open_ = true;
    std::cout << "[CustomBagReader] Opened: " << filepath << std::endl;
    
    return true;
}

bool CustomBagReader::hasNext() {
    return is_open_ && file_.peek() != EOF;
}

bool CustomBagReader::readNext() {
    if (!hasNext()) return false;
    
    uint8_t msg_type;
    file_.read(reinterpret_cast<char*>(&msg_type), 1);
    
    if (file_.eof()) return false;
    
    switch (msg_type) {
        case MSG_IMU:
            return readImuMessage();
        case MSG_POINTCLOUD:
            return readPointCloudMessage();
        default:
            // Unknown message type, try to skip
            return true;
    }
}

bool CustomBagReader::readImuMessage() {
    ImuData imu;
    
    // Read timestamp (8 bytes)
    file_.read(reinterpret_cast<char*>(&imu.timestamp), 8);
    
    // Read gyro (3 * 8 bytes)
    file_.read(reinterpret_cast<char*>(&imu.gyro_x), 8);
    file_.read(reinterpret_cast<char*>(&imu.gyro_y), 8);
    file_.read(reinterpret_cast<char*>(&imu.gyro_z), 8);
    
    // Read accel (3 * 8 bytes)
    file_.read(reinterpret_cast<char*>(&imu.accel_x), 8);
    file_.read(reinterpret_cast<char*>(&imu.accel_y), 8);
    file_.read(reinterpret_cast<char*>(&imu.accel_z), 8);
    
    if (file_.fail()) return false;
    
    dispatchImu(imu);
    return true;
}

bool CustomBagReader::readPointCloudMessage() {
    PointCloudData pc;
    
    // Read timestamp
    file_.read(reinterpret_cast<char*>(&pc.timestamp), 8);
    
    // Read point count
    uint32_t num_points;
    file_.read(reinterpret_cast<char*>(&num_points), 4);
    
    if (file_.fail()) return false;
    
    pc.points.resize(num_points);
    
    // Read points
    for (uint32_t i = 0; i < num_points; i++) {
        file_.read(reinterpret_cast<char*>(&pc.points[i].x), 4);
        file_.read(reinterpret_cast<char*>(&pc.points[i].y), 4);
        file_.read(reinterpret_cast<char*>(&pc.points[i].z), 4);
        file_.read(reinterpret_cast<char*>(&pc.points[i].reflectivity), 4);
        file_.read(reinterpret_cast<char*>(&pc.points[i].offset_time), 8);
        file_.read(reinterpret_cast<char*>(&pc.points[i].line), 1);
    }
    
    if (file_.fail()) return false;
    
    dispatchPointCloud(pc);
    return true;
}

void CustomBagReader::close() {
    if (is_open_) {
        file_.close();
        is_open_ = false;
    }
}

// ============================================================================
// Factory function
// ============================================================================

std::unique_ptr<DataReader> createDataReader(const std::string& filepath) {
    // Determine file type from extension
    std::string ext;
    size_t dot_pos = filepath.rfind('.');
    if (dot_pos != std::string::npos) {
        ext = filepath.substr(dot_pos);
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    }
    
    if (ext == ".pcap") {
        return std::make_unique<PcapReader>();
    } else if (ext == ".bag") {
        return std::make_unique<CustomBagReader>();
    } else {
        std::cerr << "[DataReader] Unknown file extension: " << ext << std::endl;
        std::cerr << "[DataReader] Trying PCAP reader..." << std::endl;
        return std::make_unique<PcapReader>();
    }
}
