#pragma once

#include <Eigen/Dense>
// %            ... representing transformation from sensor frame origin w.r.t. to body frame origin in body frame coordinate                               [m]
using PositionBody2Sensor = Eigen::Matrix4d;
// %            ... struct representing single 3d point data
struct Point3d{
    Eigen::Vector3d pointsBody = Eigen::Vector3d::Zero();           // raw point x,y,z in body frame                                                              [m]
    Eigen::Vector3d pointsMap = Eigen::Vector3d::Zero();               // storage for point x,y,z in local coordinate frame                                          [m]
    double alpha = 0.0;                                             // relative timestamp to the absolute timestamp of the frame          
    double timestamp = 0.0;                                         // absolute timestamp of the point                                                            [s]
    int beamid = -1;                                                // beam id of the point in lidar configuration                        
};
// %            ... struct representing single frame imu data
struct ImuData{
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();                  // acceleration x,y,z in IMU sensor frame                                                     [m/(s*s)]
    Eigen::Vector3d gyr = Eigen::Vector3d::Zero();                  // angular rate around x-axis,y-axis,z-axis in IMU sensor frame                               [rad/s]
    Eigen::Vector3d accStdDev = Eigen::Vector3d::Zero();            // standard deviation for acceleration x,y,z in IMU sensor frame                              [m/(s*s)]
    Eigen::Vector3d gyrStdDev = Eigen::Vector3d::Zero();            // standard deviation for angular rate around x-axis,y-axis,z-axis in IMU sensor frame        [rad/s]
    Eigen::Vector3d accBias = Eigen::Vector3d::Zero();              // bias for acceleration x,y,z in IMU sensor frame                                            [m/(s*s)]
    Eigen::Vector3d gyrBias = Eigen::Vector3d::Zero();              // bias for angular rate around x-axis,y-axis,z-axis in IMU sensor frame                      [rad/s]
    double timestamp = 0.0;                                         // absolute timestamp of the data                                                             [s]
};
// %            ... struct representing single frame in position data (GPS,GNSS,INS, etc..)
struct PositionData{
    Eigen::Vector3d pose = Eigen::Vector3d::Zero();                 // position latitude[rad],longitude[rad],altitude[m] in sensor frame                          ([rad],[rad],[m])
    Eigen::Vector3d euler = Eigen::Vector3d::Zero();                // position roll[rad],pitch[rad],yaw[rad] in sensor frame                          ([rad],[rad],[m])
    Eigen::Vector3d poseStdDev = Eigen::Vector3d::Zero();           // standard deviation in north,east,down in sensor frame   
    Eigen::Vector3d eulerStdDev = Eigen::Vector3d::Zero();          // standard deviation in north,east,down in sensor frame                                   [m]
    double timestamp = 0.0;                                         // absolute timestamp of the data                                                             [s]
};
// %            ... struct representing a lidar data and its encapsulated data of imu and position
struct FrameData{
    double timestamp;                                               // evaluation timestamp
    std::vector<Point3d> points;
    std::vector<ImuData> imu;
    std::vector<PositionData> position;
};
// &             ... mapping done offline
// %             ... struct to save the latest state
struct StateData{
    size_t timestamp = 0.0;                                            // timestamp in s
    uint32_t latitude = 0.0;                                           // latitude in radian                                     [r]
    uint32_t longitude = 0.0;                                          // longitude in radian                                    [r]
    uint32_t altitude = 0.0;                                           // altitude in meter positive upward   
    uint32_t north = 0.0;                                              // north in meter                                         [m]
    uint32_t east = 0.0;                                               // east in meter                                          [m]
    uint32_t down = 0.0;                                               // down in meter                                          [m]
    uint32_t velocityN = 0.0;                                          // north velocity                                         [r]
    uint32_t velocityE = 0.0;                                          // east velocity                                          [r]
    uint32_t velocityD = 0.0;                                          // down velocity                                          [r]
    uint32_t qw = 0.0;                                                 // Scalar part of unit quaternion.
    uint32_t qx = 0.0;                                                 // First element of vector part of unit quaternion.
    uint32_t qy = 0.0;                                                 // Second element of vector part of unit quaternion.
    uint32_t qz = 0.0;                                                 // Third element of vector part of unit quaternion.
    uint32_t accBiasy = 0.0;                                           // latitude in radian                     [r]
    uint32_t accBiasz = 0.0;                                           // latitude in radian                     [r]
    uint32_t gyrBiasx = 0.0;                                           // latitude in radian                     [r]
    uint32_t gyrBiasy = 0.0;                                           // latitude in radian                     [r]
    uint32_t gyrBiasz = 0.0;                                           // latitude in radian                     [r]
    Eigen::Matrix<uint32_t,15,15> cov;                                 // latitude in radian                     [r]
    std::vector<uint32_t> points;                                      // raw points at this time frame          
};
// %             ... struct for parameter
struct Options{
    size_t voxelsize = 1.0;
    uint32_t voxelcapacity = 20;
    size_t maxdistance = 300.0;
    uint32_t maxiteration = 500;
    size_t convergethreshold = 0.0001;
    size_t initialthreshold = 2.0;
    size_t minmotionthreshold = 0.1;
};
// %             ... struct for parameter
struct LidarFrame {
    uint16_t frame_id = 0;
    double timestamp = 0.0;                       // Current timestamp, unix timestamp (PTP sync)
    double timestamp_end = 0.0;                   // end_timestamp of current frame.
    double interframe_timedelta = 0.0;            // Time difference between first point in current frame and last point in last frame
    uint32_t numberpoints = 0;

    std::vector<double, Eigen::aligned_allocator<double>> x; // X coordinates
    std::vector<double, Eigen::aligned_allocator<double>> y; // Y coordinates
    std::vector<double, Eigen::aligned_allocator<double>> z; // Z coordinates
    std::vector<uint16_t> c_id;                     // Channel indices
    std::vector<uint16_t> m_id;                     // Measurement indices
    std::vector<double, Eigen::aligned_allocator<double>> timestamp_points; // Absolute timestamps
    std::vector<double, Eigen::aligned_allocator<double>> relative_timestamp; // Relative timestamps
    std::vector<uint16_t> reflectivity;             // Reflectivity values
    std::vector<uint16_t> signal;                   // Signal strengths
    std::vector<uint16_t> nir;                      // NIR values

    void reserve(size_t size) {
        x.reserve(size);
        y.reserve(size);
        z.reserve(size);
        c_id.reserve(size);
        m_id.reserve(size);
        timestamp_points.reserve(size);
        relative_timestamp.reserve(size);
        reflectivity.reserve(size);
        signal.reserve(size);
        nir.reserve(size);
    }

    // Clear all vectors
    void clear() {
        x.clear();
        y.clear();
        z.clear();
        c_id.clear();
        m_id.clear();
        timestamp_points.clear();
        relative_timestamp.clear();
        reflectivity.clear();
        signal.clear();
        nir.clear();
        numberpoints = 0;
    }

    std::vector<Point3d> toPoint3D() const {
        std::vector<Point3d> pointcloud;
        pointcloud.reserve(numberpoints);
        // Calculate the total duration of the frame once to avoid division by zero.
        const double frame_duration = this->timestamp_end - this->timestamp;
        for (size_t i = 0; i < numberpoints; ++i) {
            Point3d point;
            point.pointsBody = Eigen::Vector3d(this->x[i], this->y[i], this->z[i]);         // Raw sensor coordinates
            point.pointsMap = point.pointsBody;                                             // No motion correction; set equal to raw_pt
            point.timestamp = this->timestamp_points[i];                                    // Absolute timestamp for the point
            point.beamid = static_cast<int>(this->m_id[i]);                                 // Convert uint16_t to int
            if (frame_duration > 0.0) {
                const double elapsed_time = point.timestamp - this->timestamp;
                point.alpha = std::max(0.0, std::min(1.0, elapsed_time / frame_duration));
            } else {
                point.alpha = 0.0;
            }
            pointcloud.push_back(point);
        }
        return pointcloud;
    }
};





