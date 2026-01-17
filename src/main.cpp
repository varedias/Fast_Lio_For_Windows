/**
 * @file main.cpp
 * @brief FAST-LIO main program for processing LiDAR data files
 * 
 * This program reads pcap or bag files containing Livox MID360 data,
 * processes them through the FAST-LIO SLAM algorithm, and outputs
 * PCD point cloud files.
 * 
 * Usage: fastlio_process.exe <config_file> <data_file> [output_dir]
 */

#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <filesystem>
#include <atomic>
#include <csignal>
#include <fstream>
#include <sstream>
#include <regex>
#include <map>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "laserMapping.h"
#include "data_reader.h"

// Platform-specific includes
#if defined(_WIN32) || defined(_MSC_VER)
#include "compat/windows_compat.h"
#include <windows.h>
// Disable output buffering on Windows
#define DISABLE_OUTPUT_BUFFERING() setvbuf(stdout, NULL, _IONBF, 0); setvbuf(stderr, NULL, _IONBF, 0)
#else
#include <unistd.h>
#define DISABLE_OUTPUT_BUFFERING()
#endif

namespace fs = std::filesystem;

// Global flag for graceful shutdown
std::atomic<bool> g_shutdown_requested(false);

// Simple YAML parser for key: value pairs
class SimpleYamlParser {
public:
    bool load(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) return false;

        std::string line;
        while (std::getline(file, line)) {
            // Remove comments
            size_t comment_pos = line.find('#');
            if (comment_pos != std::string::npos) {
                line = line.substr(0, comment_pos);
            }
            if (line.find_first_not_of(" \t\r\n") == std::string::npos) {
                continue;
            }

            // Determine indentation
            size_t indent = line.find_first_not_of(" \t");
            if (indent == std::string::npos) continue;

            // Find key: value
            size_t colon_pos = line.find(':');
            if (colon_pos == std::string::npos) continue;
            
            std::string key = line.substr(0, colon_pos);
            std::string value = line.substr(colon_pos + 1);
            
            // Trim whitespace
            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t\r\n") + 1);

            if (value.empty()) {
                if (indent == 0) {
                    current_section_ = key;
                }
                continue;
            }

            std::string full_key = key;
            if (!current_section_.empty() && indent > 0) {
                full_key = current_section_ + "/" + key;
            } else if (indent == 0) {
                current_section_.clear();
            }

            if (!full_key.empty()) {
                values_[full_key] = value;
            }
        }
        return true;
    }
    
    int getInt(const std::string& key, int default_val) const {
        auto it = values_.find(key);
        if (it != values_.end()) {
            try { return std::stoi(it->second); } catch(...) {}
        }
        return default_val;
    }
    
    double getDouble(const std::string& key, double default_val) const {
        auto it = values_.find(key);
        if (it != values_.end()) {
            try { return std::stod(it->second); } catch(...) {}
        }
        return default_val;
    }
    
    bool getBool(const std::string& key, bool default_val) const {
        auto it = values_.find(key);
        if (it != values_.end()) {
            return it->second == "true" || it->second == "True" || it->second == "1";
        }
        return default_val;
    }

    std::vector<double> getDoubleList(const std::string& key, const std::vector<double>& default_val) const {
        auto it = values_.find(key);
        if (it == values_.end()) return default_val;

        std::string v = it->second;
        // Strip brackets if present
        if (!v.empty() && v.front() == '[') v.erase(0, 1);
        if (!v.empty() && v.back() == ']') v.pop_back();

        for (char &c : v) {
            if (c == ',') c = ' ';
        }

        std::istringstream iss(v);
        std::vector<double> out;
        double val;
        while (iss >> val) {
            out.push_back(val);
        }
        if (out.empty()) return default_val;
        return out;
    }
    
private:
    std::map<std::string, std::string> values_;
    std::string current_section_;
};

void signalHandler(int signum) {
    std::cout << "\n[INFO] Interrupt signal (" << signum << ") received." << std::endl;
    g_shutdown_requested = true;
}

void printUsage(const char* program) {
    std::cout << "========================================" << std::endl;
    std::cout << "  FAST-LIO Point Cloud Processor" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    std::cout << "Usage: " << program << " <config_file> <data_file> [output_dir]" << std::endl;
    std::cout << std::endl;
    std::cout << "Arguments:" << std::endl;
    std::cout << "  config_file  - Path to YAML configuration file (e.g., config/mid360.yaml)" << std::endl;
    std::cout << "  data_file    - Path to input data file (.pcap or .bag)" << std::endl;
    std::cout << "  output_dir   - (Optional) Output directory for PCD files (default: ./PCD)" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << program << " config/mid360.yaml data.pcap" << std::endl;
    std::cout << "  " << program << " config/mid360.yaml data.bag ./output" << std::endl;
    std::cout << std::endl;
    std::cout.flush();
}

class FastLioProcessor {
public:
    FastLioProcessor() : initialized_(false), frames_processed_(0) {}
    
    void setOutputDir(const std::string& dir) {
        output_dir_ = dir;
    }
    
    bool initialize(const std::string& config_path) {
        std::cout << "[FAST-LIO] Initializing with config: " << config_path << std::endl;
        
        // Load config from YAML file
        SimpleYamlParser yaml;
        if (yaml.load(config_path)) {
            FastLioConfig config;
            config.lidar_type = yaml.getInt("preprocess/lidar_type", 5);  // Default to MID360
            config.scan_line = yaml.getInt("preprocess/scan_line", 4);
            config.blind = yaml.getDouble("preprocess/blind", 0.5);
            config.acc_cov = yaml.getDouble("mapping/acc_cov", 0.1);
            config.gyr_cov = yaml.getDouble("mapping/gyr_cov", 0.1);
            config.b_acc_cov = yaml.getDouble("mapping/b_acc_cov", 0.0001);
            config.b_gyr_cov = yaml.getDouble("mapping/b_gyr_cov", 0.0001);
            config.fov_deg = yaml.getDouble("mapping/fov_degree", 360.0);
            config.det_range = yaml.getDouble("mapping/det_range", 100.0);
            config.extrinsic_est_en = yaml.getBool("mapping/extrinsic_est_en", false);
            config.extrinsic_T = yaml.getDoubleList("mapping/extrinsic_T", config.extrinsic_T);
            config.extrinsic_R = yaml.getDoubleList("mapping/extrinsic_R", config.extrinsic_R);
            config.time_sync_en = yaml.getBool("common/time_sync_en", false);
            config.time_offset_lidar_to_imu = yaml.getDouble("common/time_offset_lidar_to_imu", 0.0);
            int pf = yaml.getInt("point_filter_num", -1);
            if (pf < 0) pf = yaml.getInt("preprocess/point_filter_num", 1);
            config.point_filter_num = pf;
            config.pcd_save_en = yaml.getBool("pcd_save/pcd_save_en", true);
            config.pcd_save_interval = yaml.getInt("pcd_save/interval", -1);
            // Additional parameters matching original FAST-LIO
            config.filter_size_corner = yaml.getDouble("filter_size_corner", 0.5);
            config.filter_size_surf = yaml.getDouble("filter_size_surf", 0.5);
            config.filter_size_map = yaml.getDouble("filter_size_map", 0.5);
            config.cube_side_length = yaml.getDouble("cube_side_length", 200.0);
            config.scan_rate = yaml.getInt("preprocess/scan_rate", 10);
            config.timestamp_unit = yaml.getInt("preprocess/timestamp_unit", 1);
            
            set_config(config);
        } else {
            std::cerr << "[WARN] Could not load config file, using defaults" << std::endl;
        }
        
        init();
        initialized_ = true;
        
        std::cout << "[FAST-LIO] Initialization complete" << std::endl;
        return true;
    }
    
    void processImu(const sensor_msgs::Imu::ConstPtr& msg) {
        if (!initialized_) return;
        imu_cbk(msg);
        imu_count_++;
    }
    
    void processPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (!initialized_) return;
        
        standard_pcl_cbk(msg);
        
        // Run SLAM algorithm
        auto result = run();
        
        if (result.has_value()) {
            frames_processed_++;
            
            if (frames_processed_ % 10 == 0) {
                std::cout << "\r[FAST-LIO] Processed " << frames_processed_ 
                          << " frames, IMU: " << imu_count_ << " samples" << std::flush;
            }
        } else {
            // If FAST-LIO couldn't process, save raw point cloud
            raw_scan_count_++;
            if (raw_scan_count_ % 10 == 0) {  // Save every 10th scan
                saveRawPointCloud(msg);
            }
        }
    }
    
    void saveRawPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        if (output_dir_.empty()) return;
        
        // Convert to PCL point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        
        // Parse PointCloud2 message
        int x_off = -1, y_off = -1, z_off = -1, i_off = -1;
        for (const auto& f : msg->fields) {
            if (f.name == "x") x_off = f.offset;
            else if (f.name == "y") y_off = f.offset;
            else if (f.name == "z") z_off = f.offset;
            else if (f.name == "intensity") i_off = f.offset;
        }
        
        if (x_off < 0 || y_off < 0 || z_off < 0) return;
        
        cloud->resize(msg->width * msg->height);
        for (size_t i = 0; i < cloud->size(); i++) {
            const uint8_t* ptr = msg->data.data() + i * msg->point_step;
            memcpy(&cloud->points[i].x, ptr + x_off, sizeof(float));
            memcpy(&cloud->points[i].y, ptr + y_off, sizeof(float));
            memcpy(&cloud->points[i].z, ptr + z_off, sizeof(float));
            if (i_off >= 0) {
                memcpy(&cloud->points[i].intensity, ptr + i_off, sizeof(float));
            }
        }
        
        // Save to PCD file
        std::string filename = output_dir_ + "/raw_scan_" + std::to_string(raw_clouds_saved_) + ".pcd";
        pcl::io::savePCDFileBinary(filename, *cloud);
        raw_clouds_saved_++;
        
        if (raw_clouds_saved_ % 10 == 0) {
            std::cout << "\r[RAW] Saved " << raw_clouds_saved_ << " raw point clouds" << std::flush;
        }
    }
    
    void saveMap() {
        std::cout << std::endl << "[FAST-LIO] Saving map..." << std::endl;
        save_map();
        std::cout << "[FAST-LIO] Map saved!" << std::endl;
        std::cout << "[SUMMARY] FAST-LIO frames: " << frames_processed_ 
                  << ", Raw PCD saved: " << raw_clouds_saved_ << std::endl;
    }
    
    int getFramesProcessed() const { return frames_processed_; }
    int getRawCloudsSaved() const { return raw_clouds_saved_; }
    
private:
    bool initialized_;
    int frames_processed_;
    int imu_count_ = 0;
    int raw_scan_count_ = 0;
    int raw_clouds_saved_ = 0;
    std::string output_dir_;
};

int main(int argc, char** argv) {
    // Disable output buffering for immediate console output
    DISABLE_OUTPUT_BUFFERING();
    
    // Early debug output
    std::cerr << "[DEBUG] main() started, argc=" << argc << std::endl;
    for (int i = 0; i < argc; ++i) {
        std::cerr << "[DEBUG] argv[" << i << "]=" << argv[i] << std::endl;
    }
    std::cerr.flush();
    
    // Register signal handler
    signal(SIGINT, signalHandler);
#ifdef _WIN32
    signal(SIGBREAK, signalHandler);
#endif
    
    // Parse command line arguments
    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }
    
    std::string config_file = argv[1];
    std::string data_file = argv[2];
    std::string output_dir = (argc > 3) ? argv[3] : "./PCD";
    
    // Validate input files
    if (!fs::exists(config_file)) {
        std::cerr << "[ERROR] Config file not found: " << config_file << std::endl;
        return 1;
    }
    
    if (!fs::exists(data_file)) {
        std::cerr << "[ERROR] Data file not found: " << data_file << std::endl;
        return 1;
    }
    
    // Create output directory
    if (!fs::exists(output_dir)) {
        fs::create_directories(output_dir);
        std::cout << "[INFO] Created output directory: " << output_dir << std::endl;
    }
    
    std::cout << "========================================" << std::endl;
    std::cout << "  FAST-LIO Point Cloud Processor" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "[INFO] Config file: " << config_file << std::endl;
    std::cout << "[INFO] Data file:   " << data_file << std::endl;
    std::cout << "[INFO] Output dir:  " << output_dir << std::endl;
    std::cout << "========================================" << std::endl;
    
    // Create processor
    FastLioProcessor processor;
    processor.setOutputDir(output_dir);
    
    // Initialize FAST-LIO
    if (!processor.initialize(config_file)) {
        std::cerr << "[ERROR] Failed to initialize FAST-LIO" << std::endl;
        return 1;
    }
    
    // Create data reader
    auto reader = createDataReader(data_file);
    if (!reader) {
        std::cerr << "[ERROR] Failed to create data reader" << std::endl;
        return 1;
    }
    
    // Set callbacks
    reader->setImuCallback([&processor](const sensor_msgs::Imu::ConstPtr& msg) {
        processor.processImu(msg);
    });
    
    reader->setPointCloudCallback([&processor](const sensor_msgs::PointCloud2::ConstPtr& msg) {
        processor.processPointCloud(msg);
    });
    
    // Open data file
    if (!reader->open(data_file)) {
        std::cerr << "[ERROR] Failed to open data file" << std::endl;
        return 1;
    }
    
    // Process data
    std::cout << std::endl << "[INFO] Processing data..." << std::endl;
    auto start_time = std::chrono::steady_clock::now();
    
    while (reader->hasNext() && !g_shutdown_requested) {
        if (!reader->readNext()) {
            std::cerr << "[WARN] Error reading data packet" << std::endl;
        }
    }
    
    reader->close();
    
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Processing Complete!" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "[INFO] FAST-LIO frames:   " << processor.getFramesProcessed() << std::endl;
    std::cout << "[INFO] Raw clouds saved:  " << processor.getRawCloudsSaved() << std::endl;
    std::cout << "[INFO] Processing time:   " << duration.count() << " seconds" << std::endl;
    
    // Save final map
    processor.saveMap();
    
    std::cout << "[INFO] Done!" << std::endl;
    
    return 0;
}
