#pragma once

#include<map.hpp>
#include<dataframe.hpp>

#include <tbb/parallel_for.h>
#include <tbb/concurrent_hash_map.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_reduce.h>

#include <sophus/se3.hpp>

namespace Eigen {
    using Matrix6d = Eigen::Matrix<double, 6, 6>;
    using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
    using Vector6d = Eigen::Matrix<double, 6, 1>;
}
using Correspondences = tbb::concurrent_vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>;
using LinearSystem = std::pair<Eigen::Matrix6d, Eigen::Vector6d>;

class sensorfusion {
    public:
        // %            ... sensor fusion constructor
        sensorfusion(const std::string& json);
        // %            ... process pipeline
        StateData processmodel(); // in pipeline we will push output to spsc

    private:
        // %            ... pointcloud preprocessing helper
        void subsampling(std::vector<Point3d>& frame, double vs);
         // %            ... pointcloud preprocessing helper
        void gridsampling(const std::vector<Point3d>& frame, std::vector<Point3d>& keypoints, double vs);
        // %            ... initialize registration helper
        std::vector<Point3d> initframe(const std::vector<Point3d>& cframe, const Sophus::SE3d& Tbn2bp);
        // %            ... initialize registration helper
        Sophus::SE3d initmotion(const Sophus::SE3d& Tbp2m, const Sophus::SE3d& Tbn2bp);
        // %            ... initialize registration helper
        double initgravity(double lat, double lon, double alt);
        // %            ... initialize registration helper
        void updatemap(std::vector<Point3d>& frame, const Sophus::SE3d& Tb2m);
        // %            ... initialize registration helper
        void registerframe(const FrameData& cframe);
        // %            ... initialize registration helper
        Correspondences DataAssociation(const std::vector<Point3d>& points, const double maxdist);
        // %            ... initialize registration helper
        LinearSystem BuildLinearSystem(const Correspondences &corr, const double sc);
        // %            ... initialize registration helper
        Sophus::SE3d AlignPointsToMap(std::vector<Point3d>& frame, const Sophus::SE3d& Tb2m, const double maxdist, const double sc);
        // %            ... initialize registration helper
        double ComputeThreshold() const { return std::sqrt(model_sse_ / num_samples_); }
        // %            ... initialize registration helper
        void UpdateModelDeviation(const Sophus::SE3d &T);

    protected:
        // %            ... initialize registration helper
        map map_;
        
    private:
        // %            ... initialize registration helper
        Eigen::Vector3d lla2ned(double lat, double lon, double alt, double rlat, double rlon, double ralt);
        // %            ... initialize registration helper
        Eigen::Vector3d ned2lla(double n, double e, double d, double rlat, double rlon, double ralt);
        // %            ... initialize registration helper
        double SymmetricalAngle(double x);

    private:
        Options options_;
        Sophus::SE3d Tbc2m_     = Sophus::SE3d();                                       // T body current to map
        Sophus::SE3d Tbp2m_     = Sophus::SE3d();;                                      // T body previous to map
        Sophus::SE3d Tbc2bp_    = Sophus::SE3d();;                                      // T body current to T body previous equivalent to delta T.
        double tsc_             = 0.0;                                                  // timestamp current
        Eigen::Vector3d rlla_   = Eigen::Vector3d::Zero();                              // referrence LLA
        double model_sse_ = options_.initialthreshold * options_.initialthreshold;
        int num_samples_ = 0;
};