# FAST-LIO Windows 版本 (无 ROS 依赖)

本项目是 FAST-LIO 算法的 Windows MSVC 移植版本，完全移除了 ROS 依赖，可以直接在 Windows 上编译运行。支持从 Livox MID360 雷达采集的 PCAP 文件进行点云建图。

## 📋 目录

- [功能特性](#功能特性)
- [系统要求](#系统要求)
- [依赖库](#依赖库)
- [安装步骤](#安装步骤)
- [使用方法](#使用方法)
- [配置文件说明](#配置文件说明)
- [项目结构](#项目结构)
- [常见问题](#常见问题)
- [致谢](#致谢)

---

## 功能特性

- ✅ **无 ROS 依赖**: 完全独立运行，无需安装 ROS
- ✅ **Windows MSVC 支持**: 使用 Visual Studio 2022 编译
- ✅ **PCAP 文件支持**: 直接读取 Livox MID360 采集的 PCAP 文件
- ✅ **PCD 输出**: 自动保存点云地图为 PCD 格式
- ✅ **IMU 融合**: 支持 LiDAR-IMU 紧耦合状态估计

---

## 系统要求

| 项目     | 要求                      |
| -------- | ------------------------- |
| 操作系统 | Windows 10/11 (64位)      |
| 编译器   | Visual Studio 2022 (MSVC) |
| CMake    | >= 3.15                   |
| 内存     | >= 8GB RAM (推荐 16GB)    |

---

## 依赖库

### 必需依赖

| 库名称          | 版本要求            | 说明            |
| --------------- | ------------------- | --------------- |
| **PCL**   | >= 1.12 (推荐 1.15) | 点云处理库      |
| **Eigen** | >= 3.3.4 (推荐 5.0) | 线性代数库      |
| **Boost** | >= 1.74             | 通常随 PCL 安装 |

### PCL 安装 (Windows)

推荐使用 PCL 预编译包：

1. 从 [PCL Releases](https://github.com/PointCloudLibrary/pcl/releases) 下载 Windows 预编译版本
2. 解压到指定目录，例如 `E:\Library_save\USE_For_PCL\PCL\PCL 1.15.1`
3. 确保包含以下子目录：
   - `include/pcl-1.15/` - 头文件
   - `lib/` - 库文件
   - `bin/` - DLL 文件
   - `3rdParty/Boost/` - Boost 库
   - `3rdParty/Eigen/` - Eigen 库 (或单独安装)
   - `3rdParty/FLANN/` - FLANN 库
   - `3rdParty/VTK/` - VTK 库

### Eigen 安装

1. 从 [Eigen](https://eigen.tuxfamily.org/) 下载
2. Eigen 是纯头文件库，解压即可使用
3. 推荐路径：`E:\Library_save\Eigen\eigen-5.0.0`

---

## 安装步骤

### 1. 克隆项目

```bash
git clone https://github.com/your-repo/FAST_LIO_CXX_NoROS.git
cd FAST_LIO_CXX_NoROS
```

### 2. 配置库路径

编辑 `CMakeLists.txt`，修改以下路径为你的实际安装路径：

```cmake
set(LIBRARY_BASE_PATH "E:/Library_save" CACHE PATH "Base path for libraries")
set(PCL_ROOT "${LIBRARY_BASE_PATH}/USE_For_PCL/PCL/PCL 1.15.1")
set(EIGEN3_INCLUDE_DIR "${LIBRARY_BASE_PATH}/Eigen/eigen-5.0.0")
```

### 3. 编译项目

**方法一：使用批处理脚本（推荐）**

```cmd
build_windows_msvc.bat
```

**方法二：手动编译**

```cmd
# 打开 Visual Studio 2022 Developer Command Prompt
mkdir build_msvc
cd build_msvc
cmake -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release ..
nmake
```

### 4. 验证编译

编译成功后，在 `build_msvc/` 目录下会生成：

- `fastlio_process.exe` - 主程序
- `fastlio_mapping.dll` - 核心算法库
- 各种运行时 DLL 文件

---

## 使用方法

### 基本用法

```cmd
cd build_msvc
fastlio_process.exe <配置文件> <数据文件> [输出目录]
```

### 参数说明

| 参数     | 必需 | 说明                              |
| -------- | ---- | --------------------------------- |
| 配置文件 | 是   | YAML 格式的配置文件路径           |
| 数据文件 | 是   | PCAP 格式的 LiDAR 数据文件        |
| 输出目录 | 否   | PCD 文件输出目录 (默认:`./PCD`) |

### 示例

```cmd
# 处理 MID360 PCAP 数据
fastlio_process.exe ..\config\mid360.yaml ..\data.pcap

# 指定输出目录
fastlio_process.exe ..\config\mid360.yaml ..\data.pcap ..\output
```

### 输出说明

程序运行后会：

1. 在控制台显示处理进度和状态估计
2. 在输出目录生成 `scans.pcd` 点云地图文件
3. 在 `Log/` 目录保存位姿日志

---

## 配置文件说明

配置文件位于 `config/` 目录，主要参数如下：

### config/mid360.yaml

```yaml
common:
    lid_topic:  "/livox/lidar"      # LiDAR 话题 (兼容性保留)
    imu_topic:  "/livox/imu"        # IMU 话题 (兼容性保留)
    time_sync_en: false             # 时间同步开关

preprocess:
    lidar_type: 5                   # 雷达类型: 5=MID360, 4=MARSIM
    scan_line: 4                    # 扫描线数
    blind: 0.5                      # 盲区距离 (米)
    timestamp_unit: 1               # 时间戳单位: 0=秒, 1=毫秒, 2=微秒, 3=纳秒
    scan_rate: 10                   # 扫描频率 (Hz)
    point_filter_num: 1             # 点云抽稀倍数

mapping:
    acc_cov: 0.1                    # 加速度计噪声协方差
    gyr_cov: 0.1                    # 陀螺仪噪声协方差
    b_acc_cov: 0.0001               # 加速度计偏置协方差
    b_gyr_cov: 0.0001               # 陀螺仪偏置协方差
    fov_degree: 360                 # 视场角 (度)
    det_range: 100.0                # 检测距离 (米)
    extrinsic_est_en: false         # 外参估计开关
    extrinsic_T: [0.04165, 0.02326, -0.0284]  # LiDAR-IMU 平移外参
    extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]  # LiDAR-IMU 旋转外参

pcd_save:
    pcd_save_en: true               # PCD 保存开关
    interval: -1                    # 保存间隔 (-1=仅保存最终地图)
```

### 雷达类型 (lidar_type)

| 值 | 雷达类型      |
| -- | ------------- |
| 1  | Livox Avia    |
| 2  | Velodyne      |
| 3  | Ouster        |
| 4  | MARSIM (仿真) |
| 5  | Livox MID360  |

---

## 项目结构

```
FAST_LIO_CXX_NoROS-main/
├── build_windows_msvc.bat    # Windows 编译脚本
├── CMakeLists.txt            # CMake 配置文件
├── config/                   # 配置文件目录
│   ├── mid360.yaml           # MID360 配置
│   └── ...
├── include/                  # 头文件
│   ├── msg.h                 # 消息类型定义 (替代 ROS 消息)
│   ├── conversions.h         # 点云转换函数
│   ├── common_lib.h          # 通用工具
│   ├── so3_math.h            # SO3 数学库
│   ├── use-ikfom.hpp         # IKFOM 接口
│   ├── compat/               # Windows 兼容层
│   │   ├── windows_compat.h  # POSIX 兼容
│   │   └── pthread.h         # 线程兼容
│   ├── ikd-Tree/             # ikd-Tree 实现
│   └── IKFoM_toolkit/        # IKFOM 工具包
├── src/                      # 源代码
│   ├── main.cpp              # 主程序入口
│   ├── laserMapping.cpp      # 核心建图算法
│   ├── laserMapping.h        # 接口定义
│   ├── preprocess.cpp        # 点云预处理
│   ├── preprocess.h
│   ├── data_reader.cpp       # PCAP 数据读取
│   ├── data_reader.h
│   ├── conversions.cpp       # 数据转换
│   └── IMU_Processing.hpp    # IMU 处理
├── build_msvc/               # 编译输出目录
├── PCD/                      # 点云输出目录
├── Log/                      # 日志目录
└── README.md                 # 本文档
```

---

## 常见问题

### Q1: 编译时找不到 PCL/Eigen

**解决方案**: 检查 `CMakeLists.txt` 中的路径配置是否正确指向你的库安装位置。

### Q2: 运行时缺少 DLL

**解决方案**:

- 确保 PCL 的 `bin` 目录已添加到系统 PATH
- 或将所需 DLL 复制到 `build_msvc` 目录

### Q3: 点云漂移严重

**解决方案**:

- 检查 IMU 数据是否正常
- 调整 `acc_cov` 和 `gyr_cov` 参数
- 确保 `extrinsic_T` 和 `extrinsic_R` 配置正确

### Q4: PCAP 文件无法读取

**解决方案**:

- 确保 PCAP 文件是由 Livox SDK 或 Wireshark 捕获的 MID360 数据
- 检查文件是否损坏

---

## 致谢

本项目基于以下开源项目：

- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) - 香港大学 MARS 实验室
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree) - 增量式 KD 树
- [IKFoM](https://github.com/hku-mars/IKFoM) - 迭代卡尔曼滤波

**原始论文**:

- [FAST-LIO2: Fast Direct LiDAR-inertial Odometry](https://ieeexplore.ieee.org/document/9697912)
- [FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter](https://arxiv.org/abs/2010.08196)

**原作者**:

[Wei Xu 徐威](https://github.com/XW-HKU)，[Yixi Cai 蔡逸熙](https://github.com/Ecstasy-EC)，[Dongjiao He 贺东娇](https://github.com/Joanna-HE)，[Fangcheng Zhu 朱方程](https://github.com/zfc-zfc)，[Jiarong Lin 林家荣](https://github.com/ziv-lin)，[Zheng Liu 刘政](https://github.com/Zale-Liu), [Borong Yuan](https://github.com/borongyuan)

---

## 许可证

本项目遵循原 FAST-LIO 项目的开源许可证。详见 [LICENSE](LICENSE) 文件。

---
