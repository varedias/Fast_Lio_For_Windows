#include "preprocess.h"
#include <algorithm>

#define RETURN0     0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
  :feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;
  N_SCANS   = 6;
  SCAN_RATE = 10;
  group_size = 8;
  disA = 0.01;
  disA = 0.1; // B?
  p2l_ratio = 225;
  limit_maxmid =6.25;
  limit_midmin =6.25;
  limit_maxmin = 3.24;
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;
  given_offset_time = false;

  jump_up_limit = cos(jump_up_limit/180*M_PI);
  jump_down_limit = cos(jump_down_limit/180*M_PI);
  cos160 = cos(cos160/180*M_PI);
  smallp_intersect = cos(smallp_intersect/180*M_PI);
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

// void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
// {  
//   avia_handler(msg);
//   *pcl_out = pl_surf;
// }

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (time_unit)
  {
    case SEC:
      time_unit_scale = 1.e3f;
      break;
    case MS:
      time_unit_scale = 1.f;
      break;
    case US:
      time_unit_scale = 1.e-3f;
      break;
    case NS:
      time_unit_scale = 1.e-6f;
      break;
    default:
      time_unit_scale = 1.f;
      break;
  }

  switch (lidar_type)
  {
  // case OUST64:
  //   oust64_handler(msg);
  //   break;

  // case VELO16:
  //   velodyne_handler(msg);
  //   break;

  case MARSIM:
    sim_handler(msg);
    break;
  
  case MID360:  // MID360 = 5 in enum
    mid360_handler(msg);
    break;
  
  default:
    // Generic handler for standard PointCloud2 with XYZI fields
    generic_handler(msg);
    break;
  }
  
  // Debug: print point cloud range after preprocessing
  static int preprocess_debug = 0;
  if (preprocess_debug < 3 && !pl_surf.empty()) {
    float min_y = 1e9, max_y = -1e9, min_z = 1e9, max_z = -1e9;
    int min_y_idx = 0, max_y_idx = 0;
    for (size_t i = 0; i < pl_surf.points.size(); i++) {
      const auto& pt = pl_surf.points[i];
      if (pt.y < min_y) { min_y = pt.y; min_y_idx = i; }
      if (pt.y > max_y) { max_y = pt.y; max_y_idx = i; }
      if (pt.z < min_z) min_z = pt.z;
      if (pt.z > max_z) max_z = pt.z;
    }
    printf("[PREPROCESS] lidar_type=%d Y=[%.2f, %.2f] Z=[%.2f, %.2f] pts=%zu\n",
           lidar_type, min_y, max_y, min_z, max_z, pl_surf.size());
    printf("[PREPROCESS] min_y pt[%d]: x=%.3f y=%.3f z=%.3f curv=%.3f\n",
           min_y_idx, pl_surf.points[min_y_idx].x, pl_surf.points[min_y_idx].y, 
           pl_surf.points[min_y_idx].z, pl_surf.points[min_y_idx].curvature);
    printf("[PREPROCESS] max_y pt[%d]: x=%.3f y=%.3f z=%.3f curv=%.3f\n",
           max_y_idx, pl_surf.points[max_y_idx].x, pl_surf.points[max_y_idx].y,
           pl_surf.points[max_y_idx].z, pl_surf.points[max_y_idx].curvature);
    preprocess_debug++;
  }
  
  *pcl_out = pl_surf;
}

// void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
// {
//   pl_surf.clear();
//   pl_corn.clear();
//   pl_full.clear();
//   double t1 = omp_get_wtime();
//   int plsize = msg->point_num;
//   // cout<<"plsie: "<<plsize<<endl;

//   pl_corn.reserve(plsize);
//   pl_surf.reserve(plsize);
//   pl_full.resize(plsize);

//   for(int i=0; i<N_SCANS; i++)
//   {
//     pl_buff[i].clear();
//     pl_buff[i].reserve(plsize);
//   }
//   uint valid_num = 0;
  
//   if (feature_enabled)
//   {
//     for(uint i=1; i<plsize; i++)
//     {
//       if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
//       {
//         pl_full[i].x = msg->points[i].x;
//         pl_full[i].y = msg->points[i].y;
//         pl_full[i].z = msg->points[i].z;
//         pl_full[i].intensity = msg->points[i].reflectivity;
//         pl_full[i].curvature = msg->points[i].offset_time / float(1000000); //use curvature as time of each laser points

//         bool is_new = false;
//         if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
//             || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
//             || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
//         {
//           pl_buff[msg->points[i].line].push_back(pl_full[i]);
//         }
//       }
//     }
//     static int count = 0;
//     static double time = 0.0;
//     count ++;
//     double t0 = omp_get_wtime();
//     for(int j=0; j<N_SCANS; j++)
//     {
//       if(pl_buff[j].size() <= 5) continue;
//       pcl::PointCloud<PointType> &pl = pl_buff[j];
//       plsize = pl.size();
//       vector<orgtype> &types = typess[j];
//       types.clear();
//       types.resize(plsize);
//       plsize--;
//       for(uint i=0; i<plsize; i++)
//       {
//         types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
//         vx = pl[i].x - pl[i + 1].x;
//         vy = pl[i].y - pl[i + 1].y;
//         vz = pl[i].z - pl[i + 1].z;
//         types[i].dista = sqrt(vx * vx + vy * vy + vz * vz);
//       }
//       types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
//       give_feature(pl, types);
//       // pl_surf += pl;
//     }
//     time += omp_get_wtime() - t0;
//     printf("Feature extraction time: %lf \n", time / count);
//   }
//   else
//   {
//     for(uint i=1; i<plsize; i++)
//     {
//       if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
//       {
//         valid_num ++;
//         if (valid_num % point_filter_num == 0)
//         {
//           pl_full[i].x = msg->points[i].x;
//           pl_full[i].y = msg->points[i].y;
//           pl_full[i].z = msg->points[i].z;
//           pl_full[i].intensity = msg->points[i].reflectivity;
//           pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms

//           if(((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
//               || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
//               || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
//               && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)))
//           {
//             pl_surf.push_back(pl_full[i]);
//           }
//         }
//       }
//     }
//   }
// }

// void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
// {
//   pl_surf.clear();
//   pl_corn.clear();
//   pl_full.clear();
//   pcl::PointCloud<ouster_ros::Point> pl_orig = conversions::toXYZI(*msg);
//   int plsize = pl_orig.size();
//   pl_corn.reserve(plsize);
//   pl_surf.reserve(plsize);
//   if (feature_enabled)
//   {
//     for (int i = 0; i < N_SCANS; i++)
//     {
//       pl_buff[i].clear();
//       pl_buff[i].reserve(plsize);
//     }

//     for (uint i = 0; i < plsize; i++)
//     {
//       double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
//       if (range < (blind * blind)) continue;
//       Eigen::Vector3d pt_vec;
//       PointType added_pt;
//       added_pt.x = pl_orig.points[i].x;
//       added_pt.y = pl_orig.points[i].y;
//       added_pt.z = pl_orig.points[i].z;
//       added_pt.intensity = pl_orig.points[i].intensity;
//       added_pt.normal_x = 0;
//       added_pt.normal_y = 0;
//       added_pt.normal_z = 0;
//       double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
//       if (yaw_angle >= 180.0)
//         yaw_angle -= 360.0;
//       if (yaw_angle <= -180.0)
//         yaw_angle += 360.0;

//       added_pt.curvature = pl_orig.points[i].t * time_unit_scale;
//       if(pl_orig.points[i].ring < N_SCANS)
//       {
//         pl_buff[pl_orig.points[i].ring].push_back(added_pt);
//       }
//     }

//     for (int j = 0; j < N_SCANS; j++)
//     {
//       PointCloudXYZI &pl = pl_buff[j];
//       int linesize = pl.size();
//       vector<orgtype> &types = typess[j];
//       types.clear();
//       types.resize(linesize);
//       linesize--;
//       for (uint i = 0; i < linesize; i++)
//       {
//         types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
//         vx = pl[i].x - pl[i + 1].x;
//         vy = pl[i].y - pl[i + 1].y;
//         vz = pl[i].z - pl[i + 1].z;
//         types[i].dista = vx * vx + vy * vy + vz * vz;
//       }
//       types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
//       give_feature(pl, types);
//     }
//   }
//   else
//   {
//     double time_stamp = msg->header.stamp.toSec();
//     // cout << "===================================" << endl;
//     // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
//     for (int i = 0; i < pl_orig.points.size(); i++)
//     {
//       if (i % point_filter_num != 0) continue;

//       double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      
//       if (range < (blind * blind)) continue;
      
//       Eigen::Vector3d pt_vec;
//       PointType added_pt;
//       added_pt.x = pl_orig.points[i].x;
//       added_pt.y = pl_orig.points[i].y;
//       added_pt.z = pl_orig.points[i].z;
//       added_pt.intensity = pl_orig.points[i].intensity;
//       added_pt.normal_x = 0;
//       added_pt.normal_y = 0;
//       added_pt.normal_z = 0;
//       added_pt.curvature = pl_orig.points[i].t * time_unit_scale; // curvature unit: ms

//       pl_surf.points.push_back(added_pt);
//     }
//   }
//   // pub_func(pl_surf, pub_full, msg->header.stamp);
//   // pub_func(pl_surf, pub_corn, msg->header.stamp);
// }

// void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
// {
//     pl_surf.clear();
//     pl_corn.clear();
//     pl_full.clear();

//     pcl::PointCloud<velodyne_ros::Point> pl_orig = conversions::toXYZI(*msg);
//     int plsize = pl_orig.points.size();
//     if (plsize == 0) return;
//     pl_surf.reserve(plsize);

//     /*** These variables only works when no point timestamps given ***/
//     double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
//     std::vector<bool> is_first(N_SCANS,true);
//     std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
//     std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
//     std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
//     /*****************************************************************/

//     if (pl_orig.points[plsize - 1].time > 0)
//     {
//       given_offset_time = true;
//     }
//     else
//     {
//       given_offset_time = false;
//       double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
//       double yaw_end  = yaw_first;
//       int layer_first = pl_orig.points[0].ring;
//       for (uint i = plsize - 1; i > 0; i--)
//       {
//         if (pl_orig.points[i].ring == layer_first)
//         {
//           yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
//           break;
//         }
//       }
//     }

//     if(feature_enabled)
//     {
//       for (int i = 0; i < N_SCANS; i++)
//       {
//         pl_buff[i].clear();
//         pl_buff[i].reserve(plsize);
//       }
      
//       for (int i = 0; i < plsize; i++)
//       {
//         PointType added_pt;
//         added_pt.normal_x = 0;
//         added_pt.normal_y = 0;
//         added_pt.normal_z = 0;
//         int layer  = pl_orig.points[i].ring;
//         if (layer >= N_SCANS) continue;
//         added_pt.x = pl_orig.points[i].x;
//         added_pt.y = pl_orig.points[i].y;
//         added_pt.z = pl_orig.points[i].z;
//         added_pt.intensity = pl_orig.points[i].intensity;
//         added_pt.curvature = pl_orig.points[i].time * time_unit_scale; // units: ms

//         if (!given_offset_time)
//         {
//           double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
//           if (is_first[layer])
//           {
//             // printf("layer: %d; is first: %d", layer, is_first[layer]);
//               yaw_fp[layer]=yaw_angle;
//               is_first[layer]=false;
//               added_pt.curvature = 0.0;
//               yaw_last[layer]=yaw_angle;
//               time_last[layer]=added_pt.curvature;
//               continue;
//           }

//           if (yaw_angle <= yaw_fp[layer])
//           {
//             added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
//           }
//           else
//           {
//             added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
//           }

//           if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

//           yaw_last[layer] = yaw_angle;
//           time_last[layer]=added_pt.curvature;
//         }

//         pl_buff[layer].points.push_back(added_pt);
//       }

//       for (int j = 0; j < N_SCANS; j++)
//       {
//         PointCloudXYZI &pl = pl_buff[j];
//         int linesize = pl.size();
//         if (linesize < 2) continue;
//         vector<orgtype> &types = typess[j];
//         types.clear();
//         types.resize(linesize);
//         linesize--;
//         for (uint i = 0; i < linesize; i++)
//         {
//           types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
//           vx = pl[i].x - pl[i + 1].x;
//           vy = pl[i].y - pl[i + 1].y;
//           vz = pl[i].z - pl[i + 1].z;
//           types[i].dista = vx * vx + vy * vy + vz * vz;
//         }
//         types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
//         give_feature(pl, types);
//       }
//     }
//     else
//     {
//       for (int i = 0; i < plsize; i++)
//       {
//         PointType added_pt;
//         // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;
        
//         added_pt.normal_x = 0;
//         added_pt.normal_y = 0;
//         added_pt.normal_z = 0;
//         added_pt.x = pl_orig.points[i].x;
//         added_pt.y = pl_orig.points[i].y;
//         added_pt.z = pl_orig.points[i].z;
//         added_pt.intensity = pl_orig.points[i].intensity;
//         added_pt.curvature = pl_orig.points[i].time * time_unit_scale;  // curvature unit: ms // cout<<added_pt.curvature<<endl;

//         if (!given_offset_time)
//         {
//           int layer = pl_orig.points[i].ring;
//           double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

//           if (is_first[layer])
//           {
//             // printf("layer: %d; is first: %d", layer, is_first[layer]);
//               yaw_fp[layer]=yaw_angle;
//               is_first[layer]=false;
//               added_pt.curvature = 0.0;
//               yaw_last[layer]=yaw_angle;
//               time_last[layer]=added_pt.curvature;
//               continue;
//           }

//           // compute offset time
//           if (yaw_angle <= yaw_fp[layer])
//           {
//             added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
//           }
//           else
//           {
//             added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
//           }

//           if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

//           yaw_last[layer] = yaw_angle;
//           time_last[layer]=added_pt.curvature;
//         }

//         if (i % point_filter_num == 0)
//         {
//           if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind))
//           {
//             pl_surf.points.push_back(added_pt);
//           }
//         }
//       }
//     }
// }

void Preprocess::sim_handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // MARSIM handler: similar to mid360_handler but sets curvature to 0 for all points
    // This mode assumes all points are captured at the same instant, no motion distortion
    pl_surf.clear();
    pl_full.clear();
    
    int plsize = msg->width * msg->height;
    pl_surf.reserve(plsize);
    
    // Find field offsets
    int offset_x = -1, offset_y = -1, offset_z = -1, offset_intensity = -1;
    for (size_t i = 0; i < msg->fields.size(); i++) {
        if (msg->fields[i].name == "x") offset_x = msg->fields[i].offset;
        else if (msg->fields[i].name == "y") offset_y = msg->fields[i].offset;
        else if (msg->fields[i].name == "z") offset_z = msg->fields[i].offset;
        else if (msg->fields[i].name == "intensity") offset_intensity = msg->fields[i].offset;
    }
    
    if (offset_x < 0 || offset_y < 0 || offset_z < 0) {
        printf("[sim_handler] ERROR: Missing x/y/z fields!\n");
        return;
    }
    
    uint valid_num = 0;
    
    for (int i = 0; i < plsize; i++) {
        const uint8_t* ptr = &msg->data[i * msg->point_step];
        float x, y, z, intensity = 0.0f;
        
        memcpy(&x, ptr + offset_x, 4);
        memcpy(&y, ptr + offset_y, 4);
        memcpy(&z, ptr + offset_z, 4);
        if (offset_intensity >= 0) memcpy(&intensity, ptr + offset_intensity, 4);
        
        double range = x * x + y * y + z * z;
        
        // Skip blind zone
        if (range < blind * blind) continue;
        
        // Apply point filter
        valid_num++;
        if (valid_num % point_filter_num != 0) continue;
        
        PointType added_pt;
        added_pt.x = x;
        added_pt.y = y;
        added_pt.z = z;
        added_pt.intensity = intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = 0.0;  // MARSIM mode: no time offset
        
        pl_surf.points.push_back(added_pt);
    }
}

void Preprocess::mid360_handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // MID360 handler for standard XYZI point clouds (following original FAST-LIO algorithm)
    pl_surf.clear();
    pl_full.clear();
    
    int plsize = msg->width * msg->height;
    pl_surf.reserve(plsize);
    
    // Find field offsets
    int offset_x = -1, offset_y = -1, offset_z = -1, offset_intensity = -1, offset_time = -1;
    for (size_t i = 0; i < msg->fields.size(); i++) {
        if (msg->fields[i].name == "x") offset_x = msg->fields[i].offset;
        else if (msg->fields[i].name == "y") offset_y = msg->fields[i].offset;
        else if (msg->fields[i].name == "z") offset_z = msg->fields[i].offset;
        else if (msg->fields[i].name == "intensity") offset_intensity = msg->fields[i].offset;
        else if (msg->fields[i].name == "time") offset_time = msg->fields[i].offset;
    }
    
    // Debug: print field offsets and first points from raw message
    static int handler_debug = 0;
    if (handler_debug < 3) {
        printf("[mid360_handler] offset_x=%d offset_y=%d offset_z=%d offset_time=%d point_step=%d\n",
               offset_x, offset_y, offset_z, offset_time, msg->point_step);
        
        // Print first few raw points from msg
        if (plsize > 0 && offset_x >= 0 && offset_y >= 0 && offset_z >= 0) {
            for (int i = 0; i < std::min(3, plsize); i++) {
                const uint8_t* ptr = &msg->data[i * msg->point_step];
                float x, y, z;
                memcpy(&x, ptr + offset_x, 4);
                memcpy(&y, ptr + offset_y, 4);
                memcpy(&z, ptr + offset_z, 4);
                printf("[mid360_handler] raw msg point[%d]: x=%.3f y=%.3f z=%.3f\n", i, x, y, z);
            }
        }
        
        // Find and print extreme Y values in raw message
        float min_y = 1e9, max_y = -1e9;
        int min_y_raw_idx = 0, max_y_raw_idx = 0;
        for (int i = 0; i < plsize; i++) {
            const uint8_t* ptr = &msg->data[i * msg->point_step];
            float y;
            memcpy(&y, ptr + offset_y, 4);
            if (y < min_y) { min_y = y; min_y_raw_idx = i; }
            if (y > max_y) { max_y = y; max_y_raw_idx = i; }
        }
        printf("[mid360_handler] raw Y range: [%.3f, %.3f] at indices %d, %d\n", 
               min_y, max_y, min_y_raw_idx, max_y_raw_idx);
        
        // Print the extreme Y points
        {
            const uint8_t* ptr = &msg->data[min_y_raw_idx * msg->point_step];
            float x, y, z, t;
            memcpy(&x, ptr + offset_x, 4);
            memcpy(&y, ptr + offset_y, 4);
            memcpy(&z, ptr + offset_z, 4);
            memcpy(&t, ptr + offset_time, 4);
            printf("[mid360_handler] min_y raw point[%d]: x=%.3f y=%.3f z=%.3f t=%.3f\n", 
                   min_y_raw_idx, x, y, z, t);
        }
        {
            const uint8_t* ptr = &msg->data[max_y_raw_idx * msg->point_step];
            float x, y, z, t;
            memcpy(&x, ptr + offset_x, 4);
            memcpy(&y, ptr + offset_y, 4);
            memcpy(&z, ptr + offset_z, 4);
            memcpy(&t, ptr + offset_time, 4);
            printf("[mid360_handler] max_y raw point[%d]: x=%.3f y=%.3f z=%.3f t=%.3f\n",
                   max_y_raw_idx, x, y, z, t);
        }
        
        handler_debug++;
    }
    
    uint valid_num = 0;
    
    for (int i = 0; i < plsize; i++) {
        const uint8_t* ptr = &msg->data[i * msg->point_step];
        float x, y, z, intensity = 0.0f, time_offset = 0.0f;
        
        memcpy(&x, ptr + offset_x, 4);
        memcpy(&y, ptr + offset_y, 4);
        memcpy(&z, ptr + offset_z, 4);
        if (offset_intensity >= 0) memcpy(&intensity, ptr + offset_intensity, 4);
        if (offset_time >= 0) memcpy(&time_offset, ptr + offset_time, 4);
        
        double range = x * x + y * y + z * z;
        
        // Skip blind zone (following original FAST-LIO avia_handler)
        if (range < blind * blind) continue;
        
        // Apply point filter (following original FAST-LIO)
        valid_num++;
        if (valid_num % point_filter_num != 0) continue;
        
        PointType added_pt;
        added_pt.x = x;
        added_pt.y = y;
        added_pt.z = z;
        added_pt.intensity = intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = time_offset;  // Time offset in milliseconds
        
        pl_surf.points.push_back(added_pt);
    }
    
    // Sort points by time (curvature) to ensure points.back() has max time
    // This is required by sync_packages which uses points.back().curvature
    if (!pl_surf.empty()) {
        std::sort(pl_surf.points.begin(), pl_surf.points.end(),
            [](const PointType& a, const PointType& b) {
                return a.curvature < b.curvature;
            });
    }
}

void Preprocess::generic_handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // Generic handler for any PointCloud2 with XYZI fields
    pl_surf.clear();
    pl_full.clear();
    
    pcl::PointCloud<pcl::PointXYZI> pl_orig;
    conversions::fromROSMsg(*msg, pl_orig);
    
    int plsize = pl_orig.size();
    pl_surf.reserve(plsize);
    
    for (int i = 0; i < plsize; i++) {
        if (i % point_filter_num != 0) continue;
        
        double range = pl_orig.points[i].x * pl_orig.points[i].x + 
                       pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;
        
        if (range < blind * blind) continue;
        
        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = 0.0;
        
        pl_surf.points.push_back(added_pt);
    }
}

void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  int plsize = pl.size();
  int plsize2;
  if(plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;

  while(types[head].range < blind)
  {
    head++;
  }

  // Surf
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex = 0, i2;
  uint last_i = 0; uint last_i_nex = 0;
  int last_state = 0;
  int plane_type;

  for(uint i=head; i<plsize2; i++)
  {
    if(types[i].range < blind)
    {
      continue;
    }

    i2 = i;

    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);
    
    if(plane_type == 1)
    {
      for(uint j=i; j<=i_nex; j++)
      { 
        if(j!=i && j!=i_nex)
        {
          types[j].ftype = Real_Plane;
        }
        else
        {
          types[j].ftype = Poss_Plane;
        }
      }
      
      // if(last_state==1 && fabs(last_direct.sum())>0.5)
      if(last_state==1 && last_direct.norm()>0.1)
      {
        double mod = last_direct.transpose() * curr_direct;
        if(mod>-0.707 && mod<0.707)
        {
          types[i].ftype = Edge_Plane;
        }
        else
        {
          types[i].ftype = Real_Plane;
        }
      }
      
      i = i_nex - 1;
      last_state = 1;
    }
    else // if(plane_type == 2)
    {
      i = i_nex;
      last_state = 0;
    }
    // else if(plane_type == 0)
    // {
    //   if(last_state == 1)
    //   {
    //     uint i_nex_tem;
    //     uint j;
    //     for(j=last_i+1; j<=last_i_nex; j++)
    //     {
    //       uint i_nex_tem2 = i_nex_tem;
    //       Eigen::Vector3d curr_direct2;

    //       uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);

    //       if(ttem != 1)
    //       {
    //         i_nex_tem = i_nex_tem2;
    //         break;
    //       }
    //       curr_direct = curr_direct2;
    //     }

    //     if(j == last_i+1)
    //     {
    //       last_state = 0;
    //     }
    //     else
    //     {
    //       for(uint k=last_i_nex; k<=i_nex_tem; k++)
    //       {
    //         if(k != i_nex_tem)
    //         {
    //           types[k].ftype = Real_Plane;
    //         }
    //         else
    //         {
    //           types[k].ftype = Poss_Plane;
    //         }
    //       }
    //       i = i_nex_tem-1;
    //       i_nex = i_nex_tem;
    //       i2 = j-1;
    //       last_state = 1;
    //     }

    //   }
    // }

    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }

  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for(uint i=head+3; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i].ftype>=Real_Plane)
    {
      continue;
    }

    if(types[i-1].dista<1e-16 || types[i].dista<1e-16)
    {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
    Eigen::Vector3d vecs[2];

    for(int j=0; j<2; j++)
    {
      int m = -1;
      if(j == 1)
      {
        m = 1;
      }

      if(types[i+m].range < blind)
      {
        if(types[i].range > inf_bound)
        {
          types[i].edj[j] = Nr_inf;
        }
        else
        {
          types[i].edj[j] = Nr_blind;
        }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i+m].x, pl[i+m].y, pl[i+m].z);
      vecs[j] = vecs[j] - vec_a;
      
      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
      if(types[i].angle[j] < jump_up_limit)
      {
        types[i].edj[j] = Nr_180;
      }
      else if(types[i].angle[j] > jump_down_limit)
      {
        types[i].edj[j] = Nr_zero;
      }
    }

    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_zero && types[i].dista>0.0225 && types[i].dista>4*types[i-1].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_zero && types[i].edj[Next]== Nr_nor && types[i-1].dista>0.0225 && types[i-1].dista>4*types[i].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_inf)
    {
      if(edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if(types[i].edj[Prev]==Nr_inf && types[i].edj[Next]==Nr_nor)
    {
      if(edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }
     
    }
    else if(types[i].edj[Prev]>Nr_nor && types[i].edj[Next]>Nr_nor)
    {
      if(types[i].ftype == Nor)
      {
        types[i].ftype = Wire;
      }
    }
  }

  plsize2 = plsize-1;
  double ratio;
  for(uint i=head+1; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i-1].range<blind || types[i+1].range<blind)
    {
      continue;
    }
    
    if(types[i-1].dista<1e-8 || types[i].dista<1e-8)
    {
      continue;
    }

    if(types[i].ftype == Nor)
    {
      if(types[i-1].dista > types[i].dista)
      {
        ratio = types[i-1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i-1].dista;
      }

      if(types[i].intersect<smallp_intersect && ratio < smallp_ratio)
      {
        if(types[i-1].ftype == Nor)
        {
          types[i-1].ftype = Real_Plane;
        }
        if(types[i+1].ftype == Nor)
        {
          types[i+1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }

  int last_surface = -1;
  for(uint j=head; j<plsize; j++)
  {
    if(types[j].ftype==Poss_Plane || types[j].ftype==Real_Plane)
    {
      if(last_surface == -1)
      {
        last_surface = j;
      }
    
      if(j == uint(last_surface+point_filter_num-1))
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    }
    else
    {
      if(types[j].ftype==Edge_Jump || types[j].ftype==Edge_Plane)
      {
        pl_corn.push_back(pl[j]);
      }
      if(last_surface != -1)
      {
        PointType ap;
        for(uint k=last_surface; k<j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j-last_surface);
        ap.y /= (j-last_surface);
        ap.z /= (j-last_surface);
        ap.intensity /= (j-last_surface);
        ap.curvature /= (j-last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

// void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
// {
//   pl.height = 1; pl.width = pl.size();
//   sensor_msgs::PointCloud2 output;
//   pcl::toROSMsg(pl, output);
//   output.header.frame_id = "livox";
//   output.header.stamp = ct;
// }

int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  double group_dis = disA*types[i_cur].range + disB;
  group_dis = group_dis * group_dis;
  // i_nex = i_cur;

  double two_dis;
  vector<double> disarr;
  disarr.reserve(20);

  for(i_nex=i_cur; i_nex<i_cur+group_size; i_nex++)
  {
    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    disarr.push_back(types[i_nex].dista);
  }
  
  for(;;)
  {
    if((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx*vx + vy*vy + vz*vz;
    if(two_dis >= group_dis)
    {
      break;
    }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  double leng_wid = 0;
  double v1[3], v2[3];
  for(uint j=i_cur+1; j<i_nex; j++)
  {
    if((j >= pl.size()) || (i_cur >= pl.size())) break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    v2[0] = v1[1]*vz - vy*v1[2];
    v2[1] = v1[2]*vx - v1[0]*vz;
    v2[2] = v1[0]*vy - vx*v1[1];

    double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
    if(lw > leng_wid)
    {
      leng_wid = lw;
    }
  }


  if((two_dis*two_dis/leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;
  }

  uint disarrsize = disarr.size();
  for(uint j=0; j<disarrsize-1; j++)
  {
    for(uint k=j+1; k<disarrsize; k++)
    {
      if(disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  if(disarr[disarr.size()-2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  if(lidar_type==AVIA)
  {
    double dismax_mid = disarr[0]/disarr[disarrsize/2];
    double dismid_min = disarr[disarrsize/2]/disarr[disarrsize-2];

    if(dismax_mid>=limit_maxmid || dismid_min>=limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {
    double dismax_min = disarr[0] / disarr[disarrsize-2];
    if(dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  
  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}

bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  if(nor_dir == 0)
  {
    if(types[i-1].range<blind || types[i-2].range<blind)
    {
      return false;
    }
  }
  else if(nor_dir == 1)
  {
    if(types[i+1].range<blind || types[i+2].range<blind)
    {
      return false;
    }
  }
  double d1 = types[i+nor_dir-1].dista;
  double d2 = types[i+3*nor_dir-2].dista;
  double d;

  if(d1<d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

 
  if(d1>edgea*d2 || (d1-d2)>edgeb)
  {
    return false;
  }
  
  return true;
}
