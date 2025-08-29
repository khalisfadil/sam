#include<sensorfusion.hpp>

// %            ... subsampling
void sensorfusion::subsampling(std::vector<Point3d>& frame, double vs){
    if (frame.empty()) return;
    using VoxelMap = tbb::concurrent_hash_map<Voxel, Point3d, VoxelHash>;
    VoxelMap vm;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, frame.size()),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i != range.end(); ++i) {
                Voxel key = Voxel::coordinate(frame[i].pointsBody, vs);
                VoxelMap::accessor acc;
                vm.insert(acc, key);
                acc->second = frame[i];
            }
        }
    );
    frame.clear();
    frame.reserve(vm.size());
    for (const auto& pair : vm) {
        frame.push_back(pair.second);
    }
    frame.shrink_to_fit();
}
// %            ... gridsampling
void sensorfusion::gridsampling(const std::vector<Point3d>& frame, std::vector<Point3d>& keypoints, double vs) {
    if (frame.empty()) {
        keypoints.clear();
        return;
    }
    using VoxelMap = tbb::concurrent_hash_map<Voxel, Point3d, VoxelHash>;
    VoxelMap vm;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, frame.size()),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i != range.end(); ++i) {
                Voxel key = Voxel::coordinate(frame[i].pointsBody, vs);
                VoxelMap::accessor acc;
                vm.insert(acc, key);
                acc->second = frame[i];
            }
        }
    );
    keypoints.clear();
    keypoints.reserve(vm.size());
    for (const auto& pair : vm) {
        keypoints.push_back(pair.second);
    }
    keypoints.shrink_to_fit();
}
// %            ... initframe
std::vector<Point3d> sensorfusion::initframe(const std::vector<Point3d>& cframe, const Sophus::SE3d& Tbn2bp){
    std::vector<Point3d> frame = cframe;
    const double vs = options_.voxelsize;
    subsampling(frame,vs);
    const auto &xi = Tbn2bp.log();
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, frame.size()),
        [&](const tbb::blocked_range<size_t>& r) {
            for (size_t i = r.begin(); i != r.end(); ++i) {
                auto& point = frame[i];
                double alpha = point.alpha;
                const auto pose = Sophus::SE3d::exp((alpha - 1.0) * xi);
                point.pointsBody = pose * point.pointsBody;
            }
        }
    );
    return frame;
}
// %            ... initmotion
Sophus::SE3d sensorfusion::initmotion(const Sophus::SE3d& Tbp2m, const Sophus::SE3d& Tbn2bp) {
    return Tbp2m * Tbn2bp;
}
// %            ... initgravity
double sensorfusion::initgravity(double lat, double lon, double alt){
   
    double E        = 5.2185400842339e+5;  // WGS->E;
    double E2       = E*E;                 // WGS84->E * WGS84->E;
    double GM       = 3986004.418e+8;      // WGS84->GM_default;
    double a        = 6378137.0;           // WGS->a;
    double b        = 6356752.3142;        // WGS->b;
    double e2       = 6.69437999014e-3;    // WGS->e2;
    double b_over_a = 0.996647189335;      // WGS->b_over_a;
    double omega    = 7.292115e-5;         // WGS->omega_default;
    //                  ... Precalculation of sin/cos
    double sinphi = sin(lat);
    double cosphi = cos(lat);
    double sinlambda = sin(lon);
    double coslambda = cos(lon);
    double sin2phi = sinphi * sinphi;
    //                  ... Radius of Curvature in prime vertical (N) /eq. 4-15/ */
    double N = (a) / (sqrt(1.0 - (e2) * sin2phi));
    //                  ... Calculate rectangular coordinates /eq. 4-14/ */
    double x_rec = (N + alt) * cosphi * coslambda;
    double y_rec = (N + alt) * cosphi * sinlambda;
    double z_rec = ((b_over_a) * (b_over_a) * N + alt) * sinphi;
    //                  ... Calculate various parameters */
    double D = x_rec * x_rec + y_rec * y_rec + z_rec * z_rec - E2;
    double u2 = 0.5 * D * (1.0 + sqrt(1.0 + 4.0 * E2 * z_rec * z_rec / (D * D)));
    double u2E2 = u2 + E2;
    double u = sqrt(u2);
    double beta = atan(z_rec * sqrt(u2E2) / (u * sqrt(x_rec * x_rec + y_rec * y_rec)));
    double sinbeta = sin(beta);
    double cosbeta = cos(beta);
    double sin2beta = sinbeta * sinbeta;
    double cos2beta = cosbeta * cosbeta;
    double w = sqrt((u2 + E2 * sin2beta) / (u2E2));
    double q = 0.5 * ((1.0 + 3.0 * u2 / (E2)) * atan((E) / u) - 3.0 * u / (E));
    double qo = 0.5 * ((1.0 + 3.0 * (b) * (b) / (E2)) * atan((E) / (b)) - 3.0 * (b) / (E));
    double q_prime = 3.0 * ((1.0 + u2 / (E2)) * (1.0 - (u / (E)) * atan((E) / u))) - 1.0;
    double cf_u = u * cos2beta * omega * omega / w;
    double cf_beta = sqrt(u2E2) * cosbeta * sinbeta * omega * omega / w;
    double gamma_u = -(GM / u2E2 + omega * omega * (a) * (a) * (E) * q_prime * (0.5 * sin2beta - 1.0 / 6.0) / (u2E2 * qo)) / w + cf_u;
    double gamma_beta = omega * omega * (a) * (a) * q * sinbeta * cosbeta / (sqrt(u2E2) * w * qo) - cf_beta;
    double gravity = sqrt(gamma_u*gamma_u + gamma_beta*gamma_beta);
    return gravity;
}
// %            ... updatemap
void sensorfusion::updatemap(std::vector<Point3d>& frame, const Sophus::SE3d& Tb2m){
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, frame.size()),
        [&](const tbb::blocked_range<size_t>& r) {
            for (size_t i = r.begin(); i != r.end(); ++i) {
                auto& point = frame[i];
                point.pointsMap = Tb2m * point.pointsBody;
            }
        }
    );
    const Eigen::Vector3d& origin = Tb2m.translation();
    double mindist = (options_.voxelsize * options_.voxelsize) / options_.voxelcapacity;
    map_.add(frame, options_.voxelsize, mindist, options_.voxelcapacity);
    map_.remove(origin, options_.maxdistance);
}
// %            ... inittimestamp
void sensorfusion::registerframe(const FrameData& cframe) {
    tsc_ = cframe.timestamp;                                // store current evaluation timestsamp
    const auto pred_Tbc2m = initmotion(Tbp2m_,Tbc2bp_);     // initial guess
    auto frame = initframe(cframe.points,Tbc2bp_);          // subsample and deskew 
    std::vector<Point3d> kframe;
    gridsampling(frame,kframe,options_.voxelsize);
    const double sigma = ComputeThreshold();
    const auto Tbc2m = AlignPointsToMap(kframe, pred_Tbc2m, 3.0 * sigma, sigma);  // model deviation after gtsam
    // TODO process model
}
// %            ... inittimestamp
Correspondences sensorfusion::DataAssociation(const std::vector<Point3d> &pts, const double maxdist) {
    using points_iterator = std::vector<Point3d>::const_iterator;
    Correspondences pair;
    pair.reserve(pts.size());
    tbb::parallel_for(
        tbb::blocked_range<points_iterator>{pts.cbegin(), pts.cend()},
        [&](const tbb::blocked_range<points_iterator> &r) {
            std::for_each(r.begin(), r.end(), [&](const auto &pt) {
                const auto &[cn, dist] = map_.closestneighbor(pt.pointsMap);
                if (dist < maxdist) {
                    pair.emplace_back(pt.pointsMap, cn);
                }
            });
        });
    return pair;
}
// %            ... inittimestamp
LinearSystem sensorfusion::BuildLinearSystem(const Correspondences &corr, const double sc) {
    auto compute_jacobian_and_residual = [](const auto &corr) {
        const auto &[source, target] = corr;
        const Eigen::Vector3d residual = source - target;
        Eigen::Matrix3_6d J_r;
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat(source);
        return std::make_tuple(J_r, residual);
    };

    auto sum_linear_systems = [](LinearSystem a, const LinearSystem &b) {
        a.first += b.first;
        a.second += b.second;
        return a;
    };

    auto GM_weight = [&](const double &residual2) {
        return (sc*sc) / ((sc + residual2)*(sc + residual2));
    };

    using correspondence_iterator = Correspondences::const_iterator;
    const auto &[JTJ, JTr] = tbb::parallel_reduce(
        tbb::blocked_range<correspondence_iterator>{corr.cbegin(),corr.cend()},
        LinearSystem(Eigen::Matrix6d::Zero(), Eigen::Vector6d::Zero()),
        [&](const tbb::blocked_range<correspondence_iterator> &r, LinearSystem J) -> LinearSystem {
            return std::transform_reduce(
                r.begin(), r.end(), J, sum_linear_systems, [&](const auto &correspondence) {
                    const auto &[J_r, residual] = compute_jacobian_and_residual(correspondence);
                    const double w = GM_weight(residual.squaredNorm());
                    return LinearSystem(J_r.transpose() * w * J_r,        // JTJ
                                        J_r.transpose() * w * residual);  // JTr
                });
        },
        sum_linear_systems);
    return {JTJ, JTr};
}
// %            ... inittimestamp
Sophus::SE3d sensorfusion::AlignPointsToMap(std::vector<Point3d>& frame, const Sophus::SE3d& Tb2m, const double maxdist, const double sc) {
    if (map_.size() == 0) return Tb2m;
    std::vector<Point3d> source = frame;
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, frame.size()),
        [&](const tbb::blocked_range<size_t>& r) {
            for (size_t i = r.begin(); i != r.end(); ++i) {
                auto& point = frame[i];
                point.pointsMap = Tb2m * point.pointsBody;
            }
        }
    );
    Sophus::SE3d Ts2t = Sophus::SE3d();
    for (int j = 0; j < options_.maxiteration; ++j) {
        const auto corr = DataAssociation(source, maxdist);
        const auto &[JTJ, JTr] = BuildLinearSystem(corr, sc);
        const Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);
        const Sophus::SE3d Tmo2mn = Sophus::SE3d::exp(dx);
        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, frame.size()),
            [&](const tbb::blocked_range<size_t>& r) {
                for (size_t i = r.begin(); i != r.end(); ++i) {
                    auto& point = frame[i];
                    point.pointsMap = Tmo2mn * point.pointsMap;
                }
            }
        );
        Ts2t = Tmo2mn * Ts2t;
        if (dx.norm() < options_.convergethreshold) break;
    }
    // Spit the final transformation
    return Ts2t * Tb2m;
}
// %            ... inittimestamp
void sensorfusion::UpdateModelDeviation(const Sophus::SE3d& T) {
    const double model_error = [&]() {
        const double theta = Eigen::AngleAxisd(T.rotationMatrix()).angle();
        const double delta_rot = 2.0 * options_.maxdistance * std::sin(theta / 2.0);
        const double delta_trans = T.translation().norm();
        return delta_trans + delta_rot;
    }();
    if (model_error > options_.minmotionthreshold) {
        model_sse_ += model_error * model_error;
        num_samples_++;
    }
}
// %            ... inittimestamp
Eigen::Vector3d sensorfusion::lla2ned(double lat, double lon, double alt, double rlat, double rlon, double ralt) {
    // Constants according to WGS84
    constexpr double a = 6378137.0;              // Semi-major axis (m)
    constexpr double e2 = 0.00669437999014132;   // Squared eccentricity
    double dphi = lat - rlat;
    double dlam = SymmetricalAngle(lon - rlon);
    double dh = alt - ralt;
    double cp = std::cos(rlat);
    double sp = std::sin(rlat); // Fixed: was sin(originlon)
    double tmp1 = std::sqrt(1 - e2 * sp * sp);
    double tmp3 = tmp1 * tmp1 * tmp1;
    double dlam2 = dlam * dlam;   // Fixed: was dlam.*dlam
    double dphi2 = dphi * dphi;   // Fixed: was dphi.*dphi
    double E = (a / tmp1 + ralt) * cp * dlam -
            (a * (1 - e2) / tmp3 + ralt) * sp * dphi * dlam + // Fixed: was dphi.*dlam
            cp * dlam * dh;                                       // Fixed: was dlam.*dh
    double N = (a * (1 - e2) / tmp3 + ralt) * dphi +
            1.5 * cp * sp * a * e2 * dphi2 +
            sp * sp * dh * dphi +                              // Fixed: was dh.*dphi
            0.5 * sp * cp * (a / tmp1 + ralt) * dlam2;
    double D = -(dh - 0.5 * (a - 1.5 * a * e2 * cp * cp + 0.5 * a * e2 + ralt) * dphi2 -
                0.5 * cp * cp * (a / tmp1 - ralt) * dlam2);
    return Eigen::Vector3d(N, E, D);
}
// %            ... inittimestamp
Eigen::Vector3d sensorfusion::ned2lla(double n, double e, double d, double rlat, double rlon, double ralt) {
    // Constants and spheroid properties (WGS84)
    const double a = 6378137.0; // Semi-major axis (m)
    const double f = 1.0 / 298.257223563; // Flattening
    const double b = (1.0 - f) * a; // Semi-minor axis (m)
    const double e2 = f * (2.0 - f); // Square of first eccentricity
    const double ep2 = e2 / (1.0 - e2); // Square of second eccentricity
    double slat = std::sin(rlat);
    double clat = std::cos(rlat);
    double slon = std::sin(rlon);
    double clon = std::cos(rlon);
    double Nval = a / std::sqrt(1.0 - e2 * slat * slat);
    double rho = (Nval + ralt) * clat;
    double z0 = (Nval * (1.0 - e2) + ralt) * slat;
    double x0 = rho * clon;
    double y0 = rho * slon;
    double t = clat * (-d) - slat * n;
    double dz = slat * (-d) + clat * n;
    double dx = clon * t - slon * e;
    double dy = slon * t + clon * e;
    double x = x0 + dx;
    double y = y0 + dy;
    double z = z0 + dz;
    double lon = std::atan2(y, x);
    rho = std::hypot(x, y);
    double beta = std::atan2(z, (1.0 - f) * rho);
    double lat = std::atan2(z + b * ep2 * std::pow(std::sin(beta), 3),
                            rho - a * e2 * std::pow(std::cos(beta), 3));
    double betaNew = std::atan2((1.0 - f) * std::sin(lat), std::cos(lat));
    int count = 0;
    const int maxIterations = 5;
    while (std::abs(beta - betaNew) > 1e-10 && count < maxIterations) {
        beta = betaNew;
        lat = std::atan2(z + b * ep2 * std::pow(std::sin(beta), 3),
                         rho - a * e2 * std::pow(std::cos(beta), 3));
        betaNew = std::atan2((1.0 - f) * std::sin(lat), std::cos(lat));
        count++;
    }
    slat = std::sin(lat);
    Nval = a / std::sqrt(1.0 - e2 * slat * slat);
    double alt = rho * std::cos(lat) + (z + e2 * Nval * slat) * slat - Nval;
    return Eigen::Vector3d(lat, lon, alt);
}
// %            ... inittimestamp
double sensorfusion::SymmetricalAngle(double x) {
    constexpr double PI = M_PI;
    constexpr double TWO_PI = 2.0 * M_PI;
    double y = std::remainder(x, TWO_PI);
    if (y == PI) {y = -PI;}
    return y;
}

