#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include "msg.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstring>

typedef union {
    float f;
    unsigned char bytes[sizeof(float)];
} FloatBytes;

float bytes_to_float(unsigned char *bytes);

namespace conversions {
    sensor_msgs::PointCloud2 fromXYZI(pcl::PointCloud<pcl::PointXYZINormal> pcl_pc);

    pcl::PointCloud<pcl::PointXYZINormal> toXYZI(sensor_msgs::PointCloud2 pc_msg);
    
    // Convert sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZI>
    inline void fromROSMsg(const sensor_msgs::PointCloud2& msg, pcl::PointCloud<pcl::PointXYZI>& cloud) {
        cloud.clear();
        cloud.width = msg.width;
        cloud.height = msg.height;
        cloud.is_dense = msg.is_dense;
        cloud.points.resize(msg.width * msg.height);
        
        // Find field offsets
        int x_offset = -1, y_offset = -1, z_offset = -1, intensity_offset = -1;
        for (const auto& field : msg.fields) {
            if (field.name == "x") x_offset = field.offset;
            else if (field.name == "y") y_offset = field.offset;
            else if (field.name == "z") z_offset = field.offset;
            else if (field.name == "intensity") intensity_offset = field.offset;
        }
        
        if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
            // Invalid point cloud format
            cloud.clear();
            return;
        }
        
        for (size_t i = 0; i < cloud.points.size(); i++) {
            const uint8_t* ptr = msg.data.data() + i * msg.point_step;
            
            std::memcpy(&cloud.points[i].x, ptr + x_offset, sizeof(float));
            std::memcpy(&cloud.points[i].y, ptr + y_offset, sizeof(float));
            std::memcpy(&cloud.points[i].z, ptr + z_offset, sizeof(float));
            
            if (intensity_offset >= 0) {
                std::memcpy(&cloud.points[i].intensity, ptr + intensity_offset, sizeof(float));
            } else {
                cloud.points[i].intensity = 0.0f;
            }
        }
    }
}

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

#endif
