#ifndef MSG_H
#define MSG_H

#include <memory>
#include <string>
#include <cstring>
#include <ctime>
#include <cstdint>
#include <vector>

namespace fast_lio {
    struct Pose6D {
        float offset_time;  // the offset time of IMU measurement w.r.t the first lidar point
        float acc[3];       // the preintegrated total acceleration (global frame) at the Lidar origin
        float gyr[3];       // the unbiased angular velocity (body frame) at the Lidar origin
        float vel[3];       // the preintegrated velocity (global frame) at the Lidar origin
        float pos[3];       // the preintegrated position (global frame) at the Lidar origin
        float rot[9];       // the preintegrated rotation (global frame) at the Lidar origin - 3x3 matrix stored as array
    };
}

class Time {
  public:
    long sec;
    long nsec;

    Time() : sec(0), nsec(0) {};
    Time(long s, long ns) : sec(s), nsec(ns) {};

    double toSec() const {
        return static_cast<double>(this->sec) + static_cast<double>(this->nsec) / 1000000000.0;
    };

    long long toNSec() const {
        return this->sec * 1000000000LL + this->nsec;
    };

    static Time fromSec(double sec) {
        double integer_sec;
        double decimal_nsec;
        decimal_nsec = modf(sec, &integer_sec);
        return Time(static_cast<long>(integer_sec), static_cast<long>(decimal_nsec * 1000000000.0));
    }
};

struct Header {
    uint32_t seq;
    Time stamp;
    std::string frame_id;
};

namespace geometry_msgs {
    struct Vector3 {
        float x;
        float y;
        float z;
    };

    struct Quaternion {
        float x;
        float y;
        float z;
        float w;
    };

    struct Pose {
        Vector3 position;
        Quaternion orientation;
    };

    struct PoseWithCovariance {
        Pose pose;
        float covariance[36];
    };

    struct PoseStamped {
        Header header;
        Pose pose;
    };

    struct Twist {
        Vector3 linear;
        Vector3 angular;
    };

    struct TwistWithCovariance {
        Twist twist;
        float covariance[36];
    };
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
    typedef std::shared_ptr<Imu const> ImuConstPtr;

    struct PointField {
        std::string name;        // Name of field
        uint32_t offset;    // Offset from start of point struct
        uint8_t  datatype;  // Datatype enumeration, see above
        uint32_t count;     // How many elements in the field
    };

    struct PointCloud2 {
        Header header;
        uint32_t height;
        uint32_t width;
        bool is_bigendian;
        uint32_t point_step;
        uint32_t row_step;
        bool is_dense;
        std::vector<uint8_t> data;
        std::vector<PointField> fields;

        typedef std::shared_ptr<PointCloud2 const> ConstPtr;
    };
    typedef std::shared_ptr<PointCloud2 const> PointCloud2ConstPtr;
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