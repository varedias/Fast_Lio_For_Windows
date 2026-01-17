#include "conversions.h"

float bytes_to_float(unsigned char *bytes){
    FloatBytes data;
    data.bytes[0] = *(bytes);
    data.bytes[1] = *(bytes+1);
    data.bytes[2] = *(bytes+2);
    data.bytes[3] = *(bytes+3);
    return data.f;
}

namespace conversions {
    sensor_msgs::PointCloud2 fromXYZI(pcl::PointCloud<pcl::PointXYZINormal> pcl_pc) {
        const uint32_t num_points = sizeof(pcl_pc.points) / sizeof(pcl_pc.points[0]);
        std::vector<uint8_t> point_data;
        for (pcl::PointXYZINormal point: pcl_pc.points) {
            FloatBytes x, y, z, intensity;
            x.f = point.x;
            y.f = point.y;
            z.f = point.z;
            intensity.f = point.intensity;
            point_data.push_back(static_cast<uint8_t>(*x.bytes));
            point_data.push_back(static_cast<uint8_t>(*y.bytes));
            point_data.push_back(static_cast<uint8_t>(*z.bytes));
            point_data.push_back(static_cast<uint8_t>(*intensity.bytes));
        }
        return {
            {0},
            1,
            num_points,
            false,
            4 * sizeof(float),
            4 * sizeof(float) * num_points,
            false,
            point_data,
            {
                {"x", 0, 7}, // 7: FLOAT32
                {"y", 4, 7}, // 7: FLOAT32
                {"z", 8, 7}, // 7: FLOAT32
                {"intensity", 12, 7}, // 7: FLOAT32
            }
        };
    };

    pcl::PointCloud<pcl::PointXYZINormal> toXYZI(sensor_msgs::PointCloud2 pc_msg) {
        pcl::PointCloud<pcl::PointXYZINormal> pcl_pc;
        
        // Find field offsets
        int offset_x = -1, offset_y = -1, offset_z = -1, offset_intensity = -1;
        for (size_t i = 0; i < pc_msg.fields.size(); i++) {
            if (pc_msg.fields[i].name == "x") offset_x = pc_msg.fields[i].offset;
            else if (pc_msg.fields[i].name == "y") offset_y = pc_msg.fields[i].offset;
            else if (pc_msg.fields[i].name == "z") offset_z = pc_msg.fields[i].offset;
            else if (pc_msg.fields[i].name == "intensity") offset_intensity = pc_msg.fields[i].offset;
        }
        
        if (offset_x < 0 || offset_y < 0 || offset_z < 0) {
            return pcl_pc;  // Return empty cloud if missing fields
        }
        
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
            } else {
                point.intensity = 0.0f;
            }
            point.normal_x = 0;
            point.normal_y = 0;
            point.normal_z = 0;
            point.curvature = 0;
            
            pcl_pc.points.push_back(point);
        }
        
        pcl_pc.width = num_points;
        pcl_pc.height = 1;
        pcl_pc.is_dense = true;
        
        return pcl_pc;
    };
}