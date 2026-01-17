# FAST-LIO NoROS Windows 版本修改记录

> 本文档记录了相对于原始 FAST_LIO_CXX_NoROS 代码所做的所有修改，用于解决 Windows PCAP 数据处理中的严重漂移问题。

---

## 📋 修改概要

| 文件 | 修改类型 | 重要程度 | 描述 |
|------|----------|----------|------|
| `include/msg.h` | **🔴 关键修复** | ⭐⭐⭐⭐⭐ | Pose6D 结构体 rot 数组大小修复 |
| `src/preprocess.cpp` | 调试增强 | ⭐⭐⭐ | 添加 MID360 处理调试输出 |
| `src/conversions.cpp` | Bug 修复 | ⭐⭐⭐⭐ | toXYZI() 字段偏移解析修复 |
| `src/IMU_Processing.hpp` | 调试增强 | ⭐⭐ | 添加 UndistortPcl 调试输出 |
| `src/laserMapping.cpp` | 调试增强 | ⭐⭐ | 添加点云范围输出 |

---

## 🔴 关键修复 #1：Pose6D 结构体内存布局错误

### 文件：`include/msg.h`

### 问题描述
`Pose6D` 结构体中的 `rot` 数组被错误地声明为 `float rot[3]`，但实际需要存储 3×3 旋转矩阵（9个float元素）。

这个错误导致 `set_pose6d()` 函数写入 `rot[3..8]` 时发生内存越界，覆盖了相邻内存数据，最终导致 `UndistortPcl()` 读取到损坏的旋转矩阵。

### 症状
- 点云坐标从正常范围 (例如 Y=[-20m, +10m]) 被压缩到错误范围 (Y=[-0.5m, +1.6m])
- 最终输出的点云尺寸约为 6000 米（预期约 50 米）
- 旋转矩阵行向量的 norm 约为 0.1（应为 1.0）

### 修复

**修改前（第 18 行）：**
```cpp
struct Pose6D {
    float offset_time;
    float acc[3];
    float gyr[3];
    float vel[3];
    float pos[3];
    float rot[3];   // ❌ 错误：只有 3 个元素
};
```

**修改后：**
```cpp
struct Pose6D {
    float offset_time;
    float acc[3];
    float gyr[3];
    float vel[3];
    float pos[3];
    float rot[9];   // ✅ 正确：3x3 旋转矩阵需要 9 个元素
};
```

### 影响分析
- `set_pose6d()` 函数 (`common_lib.h`) 使用 `rot[i*3+j] = R(i,j)` 存储旋转矩阵
- `UndistortPcl()` 函数 (`IMU_Processing.hpp`) 使用 `R_imu << MAT_FROM_ARRAY(head->rot)` 读取
- 修复前：写入 rot[3..8] 越界，读取得到随机/损坏数据
- 修复后：正确存储和读取 3×3 旋转矩阵

---

## 🟡 Bug 修复 #2：toXYZI() 字段偏移解析

### 文件：`src/conversions.cpp`

### 问题描述
`toXYZI()` 函数在解析 PointCloud2 消息时，字段偏移量计算不正确。

### 修复内容
确保正确解析 PointCloud2 消息中各字段的偏移量：

```cpp
// 修复后的字段偏移解析
int offset_x = -1, offset_y = -1, offset_z = -1;
int offset_intensity = -1, offset_time = -1, offset_ring = -1;

for (const auto& field : msg.fields) {
    if (field.name == "x") offset_x = field.offset;
    else if (field.name == "y") offset_y = field.offset;
    else if (field.name == "z") offset_z = field.offset;
    else if (field.name == "intensity" || field.name == "reflectivity") 
        offset_intensity = field.offset;
    else if (field.name == "time" || field.name == "timestamp" || field.name == "t") 
        offset_time = field.offset;
    else if (field.name == "ring" || field.name == "line") 
        offset_ring = field.offset;
}
```

---

## 🟢 调试增强

### 1. `src/preprocess.cpp` - MID360 点云处理调试

添加了详细的调试输出，帮助追踪点云数据在预处理阶段的变化：

```cpp
// mid360_handler() 中添加
static int debug_count = 0;
if (debug_count < 3) {
    printf("[mid360_handler] offset_x=%d offset_y=%d offset_z=%d offset_time=%d point_step=%d\n",
           offset_x, offset_y, offset_z, offset_time, msg->point_step);
    // ... 打印原始点坐标和Y范围统计
}
```

输出示例：
```
[mid360_handler] raw Y range: [-20.168, 10.068] at indices 2007, 3269
[PREPROCESS] lidar_type=5 Y=[-20.17, 10.07] Z=[-1.47, 5.83] pts=3971
```

### 2. `src/IMU_Processing.hpp` - UndistortPcl 调试

添加了点云去畸变过程的调试输出：

```cpp
// UndistortPcl() 中添加
printf("[BEFORE_UNDISTORT] Y=[%.2f, %.2f] Z=[%.2f, %.2f]\n", min_y, max_y, min_z, max_z);
printf("[UNDISTORT] imu_state.rot:\n  [%.4f, %.4f, %.4f]\n  ...\n", ...);
printf("[UNDISTORT] head->rot raw: [%.4f, %.4f, ...]\n", head->rot[0], ...);
```

### 3. `src/laserMapping.cpp` - 点云范围输出

在 `save_map()` 函数中添加最终点云坐标范围输出：

```cpp
printf("[CLOUD_RANGE] X=[%.2f, %.2f] span=%.2f\n", min_x, max_x, max_x - min_x);
printf("[CLOUD_RANGE] Y=[%.2f, %.2f] span=%.2f\n", min_y, max_y, max_y - min_y);
printf("[CLOUD_RANGE] Z=[%.2f, %.2f] span=%.2f\n", min_z, max_z, max_z - min_z);
```

---

## 📊 修复效果对比

### 修复前

| 指标 | 值 | 状态 |
|------|-----|------|
| 点云 X 跨度 | ~6000m | ❌ 严重错误 |
| 点云 Y 跨度 | 巨大漂移 | ❌ 严重错误 |
| 点云 Z 跨度 | 巨大漂移 | ❌ 严重错误 |
| 旋转矩阵行向量 norm | ~0.1 | ❌ 无效矩阵 |
| 位置估计 | 完全错误 | ❌ |

### 修复后

| 指标 | 值 | 状态 |
|------|-----|------|
| 点云 X 跨度 | 14.66m | ✅ 正常 |
| 点云 Y 跨度 | 50.25m | ✅ 正常 |
| 点云 Z 跨度 | 60.47m | ✅ 正常 |
| 旋转矩阵行向量 norm | 1.0 | ✅ 有效矩阵 |
| 位置估计 | pos=(0.10, 0.95, 0.21) | ✅ 合理 |

---

## ⚠️ 注意事项

### 算法完整性
**本次修复没有修改 FAST-LIO 的核心算法逻辑**，所有修改仅限于：
1. 修复数据结构定义错误
2. 修复数据解析错误
3. 添加调试输出

EKF 预测、点云配准、IKD-Tree 等核心算法保持原样。

### 兼容性
- 此修复针对 Windows MSVC 编译的 NoROS 版本
- 使用 MID360 LiDAR 的 PCAP 数据
- `lidar_type = 5` (MID360 模式)

---

## 📁 相关文件列表

```
include/
├── msg.h              # [关键修复] Pose6D 结构体
├── common_lib.h       # set_pose6d() 函数（未修改，但相关）
└── conversions.h      # 点云转换声明

src/
├── preprocess.cpp     # [调试] mid360_handler
├── conversions.cpp    # [Bug修复] toXYZI()
├── IMU_Processing.hpp # [调试] UndistortPcl
└── laserMapping.cpp   # [调试] save_map

config/
└── mid360.yaml        # MID360 配置文件
```

---

## 🔍 问题根因分析

### 内存布局图示

**修复前的 Pose6D 内存布局（错误）：**
```
offset:  0    4    8   12   16   20   24   28   32   36   40   44   48   52   56
        [time][acc0][acc1][acc2][gyr0][gyr1][gyr2][vel0][vel1][vel2][pos0][pos1][pos2][rot0][rot1][rot2]
                                                                                       ↑
                                                                          只分配到这里，共 60 bytes
```

**set_pose6d() 写入时：**
```
rot[0] = R(0,0)  → 写入 offset 52  ✓
rot[1] = R(0,1)  → 写入 offset 56  ✓  
rot[2] = R(0,2)  → 写入 offset 60  ✗ 越界！
rot[3] = R(1,0)  → 写入 offset 64  ✗ 越界！
...
rot[8] = R(2,2)  → 写入 offset 84  ✗ 越界！
```

**修复后的 Pose6D 内存布局（正确）：**
```
offset:  0    4    8   12   16   20   24   28   32   36   40   44   48   52   56   60   64   68   72   76   80   84
        [time][acc0][acc1][acc2][gyr0][gyr1][gyr2][vel0][vel1][vel2][pos0][pos1][pos2][rot0][rot1][rot2][rot3][rot4][rot5][rot6][rot7][rot8]
                                                                                                                                    ↑
                                                                                                              正确分配，共 88 bytes
```

---

## 📅 修改历史

| 日期 | 版本 | 修改内容 |
|------|------|----------|
| 2026-01-17 | v1.0 | 初始修复：Pose6D.rot[3] → rot[9] |
| 2026-01-17 | v1.0 | Bug修复：toXYZI() 字段偏移解析 |
| 2026-01-17 | v1.0 | 添加调试输出 |

---

## 👤 作者

本修复由 GitHub Copilot 协助完成，用于解决 Windows 平台 MID360 PCAP 数据处理中的严重漂移问题。

