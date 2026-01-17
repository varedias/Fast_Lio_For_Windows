# FAST-LIO NoROS → Windows MSVC 完整移植指南

本文档详细记录了将 FAST-LIO C++ (NoROS版本) 从原始代码移植到 Windows MSVC 可编译版本的所有修改。

## 📌 修改概览

| 类别 | 文件 | 修改类型 |
|------|------|---------|
| 构建系统 | `CMakeLists.txt` | 重写 |
| 构建系统 | `build_windows_msvc.bat` | 新建 |
| 消息类型 | `include/msg.h` | 重写 |
| 数据转换 | `include/conversions.h` | 新建 |
| 数据转换 | `src/conversions.cpp` | 新建 |
| 数据读取 | `src/data_reader.h` | 新建 |
| 数据读取 | `src/data_reader.cpp` | 新建 |
| 主程序 | `src/main.cpp` | 新建 |
| 接口定义 | `src/laserMapping.h` | 新建 |
| 核心算法 | `src/laserMapping.cpp` | 修改 |
| 预处理 | `src/preprocess.cpp` | 修改 |
| 预处理 | `src/preprocess.h` | 修改 |
| Windows兼容 | `include/compat/windows_compat.h` | 新建 |
| Windows兼容 | `include/compat/pthread.h` | 新建 |
| Windows兼容 | `include/compat/pcl_msvc_compat.h` | 新建 |

---

## 🔧 1. 构建系统修改

### 1.1 CMakeLists.txt (完全重写)

**原始版本**: 使用 catkin (ROS) 构建系统

**新版本**: 独立 CMake 构建，支持 Windows MSVC

```cmake
cmake_minimum_required(VERSION 3.15)
project(fast_lio CXX)

# Windows/MSVC 特殊配置
if(WIN32)
    add_definitions(-DWIN32 -D_WIN32 -DWINDOWS_COMPAT -DNOMINMAX)
    add_definitions(-DPCL_SILENCE_MALLOC_WARNING=1)
    add_definitions(-DEIGEN_MAX_ALIGN_BYTES=32)
    add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
    add_definitions(-DPCL_NO_PRECOMPILE)
    
    if(MSVC)
        add_definitions(-D_CRT_SECURE_NO_WARNINGS)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2 /EHsc /MP /openmp")
    endif()
    
    # 库路径配置 (需根据实际路径调整)
    set(PCL_ROOT "path/to/PCL")
    set(EIGEN3_INCLUDE_DIR "path/to/Eigen")
    # ... 更多路径配置
endif()
```

**关键变更**:
- 移除 `catkin_package()` 和 ROS 依赖
- 添加 Windows 预定义宏
- 配置 PCL、Eigen、Boost 的 Windows 路径
- 使用 `/openmp` 替代 `-fopenmp`

### 1.2 build_windows_msvc.bat (新建)

```bat
@echo off
REM FAST-LIO Windows Build Script (MSVC)

REM 自动查找 Visual Studio 2022
if exist "D:\Visual_Studio2022\VC\Auxiliary\Build\vcvars64.bat" (
    call "D:\Visual_Studio2022\VC\Auxiliary\Build\vcvars64.bat"
)

set BUILD_DIR=%~dp0build_msvc
mkdir "%BUILD_DIR%" 2>nul
cd /d "%BUILD_DIR%"

cmake -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release "%~dp0"
nmake
```

---

## 🔧 2. 消息类型系统 (ROS-Free)

### 2.1 include/msg.h (完全重写)

**原始版本**: 使用 ROS 消息类型 (`sensor_msgs/Imu.h`, `sensor_msgs/PointCloud2.h` 等)

**新版本**: 自定义消息类型，无 ROS 依赖

```cpp
#ifndef MSG_H
#define MSG_H

#include <memory>
#include <string>
#include <vector>

namespace fast_lio {
    struct Pose6D {
        float offset_time;
        float acc[3];
        float gyr[3];
        float vel[3];
        float pos[3];
        float rot[9];  // ⚠️ 关键修复: 必须是 rot[9]，不是 rot[3]
    };
}

// Time 类 (替代 ros::Time)
class Time {
public:
    long sec;
    long nsec;
    
    Time() : sec(0), nsec(0) {}
    Time(long s, long ns) : sec(s), nsec(ns) {}
    
    double toSec() const {
        return static_cast<double>(sec) + static_cast<double>(nsec) / 1e9;
    }
    
    static Time fromSec(double sec) {
        double integer_sec;
        double decimal_nsec = modf(sec, &integer_sec);
        return Time((long)integer_sec, (long)(decimal_nsec * 1e9));
    }
};

struct Header {
    uint32_t seq;
    Time stamp;
    std::string frame_id;
};

namespace geometry_msgs {
    struct Vector3 { float x, y, z; };
    struct Quaternion { float x, y, z, w; };
    struct Pose { Vector3 position; Quaternion orientation; };
    struct PoseWithCovariance { Pose pose; float covariance[36]; };
    struct PoseStamped { Header header; Pose pose; };
    struct Twist { Vector3 linear; Vector3 angular; };
    struct TwistWithCovariance { Twist twist; float covariance[36]; };
}

namespace sensor_msgs {
    struct Imu {
        Header header;
        geometry_msgs::Quaternion orientation;
        float orientation_covariance[9];
        geometry_msgs::Vector3 angular_velocity;
        float angular_velocity_covariance[9];
        geometry_msgs::Vector3 linear_acceleration;
        float linear_acceleration_covariance[9];
        
        typedef std::shared_ptr<Imu> Ptr;
        typedef std::shared_ptr<Imu const> ConstPtr;
    };
    
    struct PointField {
        std::string name;
        uint32_t offset;
        uint8_t datatype;
        uint32_t count;
    };
    
    struct PointCloud2 {
        Header header;
        uint32_t height, width;
        bool is_bigendian;
        uint32_t point_step, row_step;
        bool is_dense;
        std::vector<uint8_t> data;
        std::vector<PointField> fields;
        
        typedef std::shared_ptr<PointCloud2 const> ConstPtr;
    };
}

namespace nav_msgs {
    struct Path {
        Header header;
        std::vector<geometry_msgs::PoseStamped> poses;
    };
    
    struct Odometry {
        Header header;
        std::string child_frame_id;
        geometry_msgs::PoseWithCovariance pose;
        geometry_msgs::TwistWithCovariance twist;
    };
}

#endif
```

### ⚠️ 关键 Bug 修复: Pose6D.rot[9]

**问题**: 原始代码中 `Pose6D.rot[3]` 只分配了 3 个 float，但 `set_pose6d()` 函数写入 9 个值（3x3 旋转矩阵）

**后果**: 内存越界，导致旋转矩阵损坏，点云漂移高达 6000m

**修复**: 将 `rot[3]` 改为 `rot[9]`

---

## 🔧 3. 数据转换模块 (新建)

### 3.1 include/conversions.h

```cpp
#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include "msg.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace conversions {
    // sensor_msgs::PointCloud2 ↔ pcl::PointCloud 转换
    sensor_msgs::PointCloud2 fromXYZI(pcl::PointCloud<pcl::PointXYZINormal> pcl_pc);
    pcl::PointCloud<pcl::PointXYZINormal> toXYZI(sensor_msgs::PointCloud2 pc_msg);
    
    // 内联转换函数
    inline void fromROSMsg(const sensor_msgs::PointCloud2& msg, 
                           pcl::PointCloud<pcl::PointXYZI>& cloud);
}

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

#endif
```

### 3.2 src/conversions.cpp

```cpp
#include "conversions.h"

namespace conversions {
    pcl::PointCloud<pcl::PointXYZINormal> toXYZI(sensor_msgs::PointCloud2 pc_msg) {
        pcl::PointCloud<pcl::PointXYZINormal> pcl_pc;
        
        // 动态查找字段偏移量
        int offset_x = -1, offset_y = -1, offset_z = -1, offset_intensity = -1;
        for (const auto& field : pc_msg.fields) {
            if (field.name == "x") offset_x = field.offset;
            else if (field.name == "y") offset_y = field.offset;
            else if (field.name == "z") offset_z = field.offset;
            else if (field.name == "intensity") offset_intensity = field.offset;
        }
        
        if (offset_x < 0 || offset_y < 0 || offset_z < 0) return pcl_pc;
        
        int num_points = pc_msg.width * pc_msg.height;
        pcl_pc.reserve(num_points);
        
        for (int i = 0; i < num_points; i++) {
            const uint8_t* ptr = &pc_msg.data[i * pc_msg.point_step];
            pcl::PointXYZINormal point;
            
            memcpy(&point.x, ptr + offset_x, sizeof(float));
            memcpy(&point.y, ptr + offset_y, sizeof(float));
            memcpy(&point.z, ptr + offset_z, sizeof(float));
            if (offset_intensity >= 0) {
                memcpy(&point.intensity, ptr + offset_intensity, sizeof(float));
            }
            point.normal_x = point.normal_y = point.normal_z = 0;
            point.curvature = 0;
            
            pcl_pc.points.push_back(point);
        }
        
        pcl_pc.width = num_points;
        pcl_pc.height = 1;
        pcl_pc.is_dense = true;
        
        return pcl_pc;
    }
}
```

---

## 🔧 4. PCAP 数据读取模块 (新建)

### 4.1 src/data_reader.h

```cpp
#ifndef DATA_READER_H
#define DATA_READER_H

#include <string>
#include <vector>
#include <deque>
#include <fstream>
#include <functional>
#include "msg.h"

struct LivoxPoint {
    float x, y, z, reflectivity;
    uint8_t tag, line;
    double offset_time;  // ns offset from scan start
};

struct ImuData {
    double timestamp;
    double gyro_x, gyro_y, gyro_z;
    double accel_x, accel_y, accel_z;
};

struct PointCloudData {
    double timestamp;
    std::vector<LivoxPoint> points;
};

using ImuCallback = std::function<void(const sensor_msgs::Imu::ConstPtr&)>;
using PointCloudCallback = std::function<void(const sensor_msgs::PointCloud2::ConstPtr&)>;

class DataReader {
public:
    virtual ~DataReader() = default;
    virtual bool open(const std::string& filepath) = 0;
    virtual bool hasNext() = 0;
    virtual bool readNext() = 0;
    virtual void close() = 0;
    
    void setImuCallback(ImuCallback cb) { imu_callback_ = cb; }
    void setPointCloudCallback(PointCloudCallback cb) { pointcloud_callback_ = cb; }
    
protected:
    ImuCallback imu_callback_;
    PointCloudCallback pointcloud_callback_;
    void dispatchImu(const ImuData& imu);
    void dispatchPointCloud(const PointCloudData& pc);
};

class PcapReader : public DataReader {
public:
    bool open(const std::string& filepath) override;
    bool hasNext() override;
    bool readNext() override;
    void close() override;
    
private:
    std::ifstream file_;
    // ... PCAP 解析相关成员
    const double SCAN_PERIOD = 0.1;  // 100ms = 10Hz
    
    bool parseLivoxPacket(const uint8_t* data, size_t len);
    bool parseLivoxImuPacket(const uint8_t* data, size_t len);
};
```

### 4.2 关键实现细节

**PCAP 解析**:
- 支持 Livox MID360 UDP 包格式
- 按 100ms 帧周期分割点云
- IMU 和 LiDAR 数据同步
- 时间戳对齐处理

---

## 🔧 5. 主程序入口 (新建)

### 5.1 src/main.cpp

```cpp
#include <iostream>
#include <filesystem>
#include "laserMapping.h"
#include "data_reader.h"

// 简单 YAML 解析器
class SimpleYamlParser {
public:
    bool load(const std::string& filepath);
    int getInt(const std::string& key, int default_val) const;
    double getDouble(const std::string& key, double default_val) const;
    bool getBool(const std::string& key, bool default_val) const;
    std::vector<double> getDoubleList(const std::string& key, const std::vector<double>& default_val) const;
};

class FastLioProcessor {
public:
    bool initialize(const std::string& config_path);
    void processImu(const sensor_msgs::Imu::ConstPtr& imu);
    void processPointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    bool runOnce();
    void saveMap();
};

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <config.yaml> <data.pcap> [output_dir]" << std::endl;
        return 1;
    }
    
    FastLioProcessor processor;
    processor.initialize(argv[1]);
    
    // 创建数据读取器
    auto reader = std::make_unique<PcapReader>();
    reader->open(argv[2]);
    
    // 设置回调
    reader->setImuCallback([&](auto imu) { processor.processImu(imu); });
    reader->setPointCloudCallback([&](auto cloud) { processor.processPointCloud(cloud); });
    
    // 处理数据
    while (reader->hasNext()) {
        reader->readNext();
        processor.runOnce();
    }
    
    processor.saveMap();
    return 0;
}
```

---

## 🔧 6. 核心算法修改

### 6.1 src/laserMapping.h (新建)

```cpp
#ifndef INTERFACE_H
#define INTERFACE_H

#include <optional>
#include "msg.h"

#if defined(_WIN32)
  #define FASTLIO_EXPORT __declspec(dllexport)
#else
  #define FASTLIO_EXPORT __attribute__((visibility("default")))
#endif

struct LidarOutput {
    std::optional<sensor_msgs::PointCloud2> pubLaserCloudFull;
    std::optional<nav_msgs::Odometry> pubOdomAftMapped;
    std::optional<nav_msgs::Path> pubPath;
};

struct FastLioConfig {
    int lidar_type = 5;  // MID360
    int scan_line = 4;
    double blind = 0.5;
    double acc_cov = 0.1;
    double gyr_cov = 0.1;
    // ... 更多配置项
};

FASTLIO_EXPORT void set_config(const FastLioConfig& config);
FASTLIO_EXPORT void init();
FASTLIO_EXPORT std::optional<LidarOutput> run();
FASTLIO_EXPORT void save_map();
FASTLIO_EXPORT void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
FASTLIO_EXPORT void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);

#endif
```

### 6.2 src/laserMapping.cpp 修改

**主要修改**:

1. **移除 ROS 依赖**:
```cpp
// 移除
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

// 替换为
#include "msg.h"
#include "conversions.h"
```

2. **添加 Windows 兼容**:
```cpp
#if defined(_WIN32) || defined(_MSC_VER)
#include "compat/windows_compat.h"
#else
#include <err.h>
#include <unistd.h>
#endif
```

3. **添加配置系统**:
```cpp
static FastLioConfig g_config;
static bool g_config_set = false;

void set_config(const FastLioConfig& config) {
    g_config = config;
    g_config_set = true;
}

void init() {
    // 应用配置
    if (g_config_set) {
        lidar_type = g_config.lidar_type;
        p_pre->blind = g_config.blind;
        // ... 更多配置应用
    }
}
```

4. **移除 ROS 节点句柄和发布者**:
```cpp
// 移除
ros::Publisher pub_path;
ros::Publisher pub_odom;

// 改为直接返回结果
std::optional<LidarOutput> run() {
    LidarOutput output;
    // ... 处理逻辑
    return output;
}
```

---

## 🔧 7. 预处理模块修改

### 7.1 src/preprocess.cpp 修改

**新增处理器**:

1. **MID360 处理器** (lidar_type = 5):
```cpp
void Preprocess::mid360_handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pl_surf.clear();
    
    // 动态查找字段偏移量
    int offset_x = -1, offset_y = -1, offset_z = -1, offset_time = -1;
    for (const auto& field : msg->fields) {
        if (field.name == "x") offset_x = field.offset;
        else if (field.name == "y") offset_y = field.offset;
        else if (field.name == "z") offset_z = field.offset;
        else if (field.name == "time") offset_time = field.offset;
    }
    
    for (int i = 0; i < plsize; i++) {
        const uint8_t* ptr = &msg->data[i * msg->point_step];
        float x, y, z, time_offset;
        
        memcpy(&x, ptr + offset_x, 4);
        memcpy(&y, ptr + offset_y, 4);
        memcpy(&z, ptr + offset_z, 4);
        if (offset_time >= 0) memcpy(&time_offset, ptr + offset_time, 4);
        
        // 过滤盲区
        if (x*x + y*y + z*z < blind*blind) continue;
        
        PointType pt;
        pt.x = x; pt.y = y; pt.z = z;
        pt.curvature = time_offset;  // 时间偏移 (ms)
        pl_surf.push_back(pt);
    }
    
    // 按时间排序
    std::sort(pl_surf.begin(), pl_surf.end(), 
        [](const PointType& a, const PointType& b) {
            return a.curvature < b.curvature;
        });
}
```

2. **MARSIM 处理器** (lidar_type = 4):
```cpp
void Preprocess::sim_handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // 类似 mid360_handler，但 curvature = 0 (无运动补偿)
    // ...
    pt.curvature = 0.0;  // MARSIM 模式无时间偏移
}
```

3. **通用处理器**:
```cpp
void Preprocess::generic_handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // 处理标准 XYZI 格式点云
}
```

### 7.2 src/preprocess.h 修改

```cpp
// 添加新的 lidar_type 枚举
enum LID_TYPE {
    AVIA = 1,
    VELO16,
    OUST64,
    MARSIM,   // 4 - 仿真模式
    MID360    // 5 - Livox MID360
};

// 添加新处理器声明
void mid360_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
void sim_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
void generic_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
```

---

## 🔧 8. Windows 兼容层 (新建)

### 8.1 include/compat/windows_compat.h

```cpp
#ifndef WINDOWS_COMPAT_H
#define WINDOWS_COMPAT_H

#if defined(_WIN32) || defined(_MSC_VER)

#include <windows.h>
#include <io.h>
#include <direct.h>

// POSIX 函数映射
#define getpid _getpid
#define access _access
#define getcwd _getcwd
#define chdir _chdir
#define isatty _isatty
#define fileno _fileno

// 文件访问模式
#define F_OK 0
#define R_OK 4
#define W_OK 2
#define X_OK 1

// Sleep 函数
static inline void usleep(unsigned int us) {
    if (us >= 1000) Sleep(us / 1000);
    else if (us > 0) Sleep(1);
}

static inline unsigned int sleep(unsigned int seconds) {
    Sleep(seconds * 1000);
    return 0;
}

// BSD err.h 替代
static inline void warn(const char *fmt, ...) {
    va_list ap;
    fprintf(stderr, "[WARN] ");
    if (fmt) {
        va_start(ap, fmt);
        vfprintf(stderr, fmt, ap);
        va_end(ap);
    }
    fprintf(stderr, "\n");
}

#endif // _WIN32
#endif // WINDOWS_COMPAT_H
```

### 8.2 include/compat/pthread.h

```cpp
#ifndef PTHREAD_COMPAT_H
#define PTHREAD_COMPAT_H

#if defined(_WIN32) || defined(_MSC_VER)

#include <windows.h>

typedef HANDLE pthread_t;
typedef CRITICAL_SECTION pthread_mutex_t;
typedef CONDITION_VARIABLE pthread_cond_t;

// 线程函数
static inline int pthread_create(pthread_t* thread, void* attr, 
                                  void*(*func)(void*), void* arg) {
    *thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)func, arg, 0, NULL);
    return (*thread == NULL) ? -1 : 0;
}

static inline int pthread_join(pthread_t thread, void** retval) {
    WaitForSingleObject(thread, INFINITE);
    CloseHandle(thread);
    return 0;
}

// 互斥锁函数
static inline int pthread_mutex_init(pthread_mutex_t* mutex, void* attr) {
    InitializeCriticalSection(mutex);
    return 0;
}

static inline int pthread_mutex_lock(pthread_mutex_t* mutex) {
    EnterCriticalSection(mutex);
    return 0;
}

static inline int pthread_mutex_unlock(pthread_mutex_t* mutex) {
    LeaveCriticalSection(mutex);
    return 0;
}

#endif // _WIN32
#endif // PTHREAD_COMPAT_H
```

### 8.3 include/compat/pcl_msvc_compat.h

```cpp
#ifndef PCL_MSVC_COMPAT_H
#define PCL_MSVC_COMPAT_H

#if defined(_MSC_VER) && _MSC_VER >= 1940
// VS2022 17.10+ 兼容性修复
#ifndef PCL_FORCE_STANDARD_OSTREAM
#define PCL_FORCE_STANDARD_OSTREAM 1
#endif
#endif

// 抑制警告
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4819)  // 非 ASCII 字符
#pragma warning(disable: 4267)  // size_t 转换
#pragma warning(disable: 4244)  // 数据丢失
#endif

#endif // PCL_MSVC_COMPAT_H
```

---

## 🔧 9. 配置文件说明

### 9.1 config/mid360.yaml

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/livox/imu"
    time_sync_en: false
    
preprocess:
    lidar_type: 5                # MID360
    scan_line: 4
    blind: 0.5
    timestamp_unit: 1            # 0=SEC, 1=MS, 2=US, 3=NS
    scan_rate: 10
    point_filter_num: 1

mapping:
    acc_cov: 0.1
    gyr_cov: 0.1
    b_acc_cov: 0.0001
    b_gyr_cov: 0.0001
    fov_degree:    360
    det_range:     100.0
    extrinsic_est_en:  false
    extrinsic_T: [ 0.04165, 0.02326, -0.0284 ]
    extrinsic_R: [ 1, 0, 0, 
                   0, 1, 0, 
                   0, 0, 1 ]

publish:
    path_en:  true
    scan_publish_en:  true
    dense_publish_en: true
    scan_bodyframe_pub_en: true

pcd_save:
    pcd_save_en: true
    interval: -1
```

---

## ✅ 10. 编译和运行

### 10.1 依赖项

- Visual Studio 2022 (MSVC)
- PCL 1.15.x (Windows 预编译版)
- Eigen 5.0.0
- Boost (通常随 PCL 安装)

### 10.2 编译步骤

```bat
cd F:\Learning\MID360\NewBuilder\FAST_LIO_CXX_NoROS-main
build_windows_msvc.bat
```

### 10.3 运行

```bat
cd build_msvc
fastlio_process.exe ..\config\mid360.yaml ..\data.pcap
```

---

## 🐛 11. 已知问题和修复

### 11.1 致命 Bug: Pose6D.rot[9] (已修复)

**问题描述**: `rot[3]` 应为 `rot[9]`，导致 3x3 旋转矩阵写入时内存越界

**症状**: 点云漂移 6000m+，旋转矩阵包含 NaN 或极大值

**修复**: 在 `include/msg.h` 中将 `float rot[3]` 改为 `float rot[9]`

### 11.2 时间戳单位问题 (已处理)

MID360 的 `curvature` 字段存储时间偏移（毫秒），需要正确处理时间单位转换。

### 11.3 点云排序 (已处理)

`sync_packages()` 依赖 `points.back().curvature` 获取最大时间偏移，因此在 `mid360_handler` 中需要对点按时间排序。

---

## 📋 12. 文件清单

### 新建文件
- `build_windows_msvc.bat`
- `include/msg.h`
- `include/conversions.h`
- `include/compat/windows_compat.h`
- `include/compat/pthread.h`
- `include/compat/pcl_msvc_compat.h`
- `src/conversions.cpp`
- `src/data_reader.h`
- `src/data_reader.cpp`
- `src/main.cpp`
- `src/laserMapping.h`

### 修改文件
- `CMakeLists.txt`
- `src/laserMapping.cpp`
- `src/preprocess.cpp`
- `src/preprocess.h`

---

**文档版本**: 1.0  
**最后更新**: 2025-01  
**作者**: FAST-LIO Windows 移植团队
