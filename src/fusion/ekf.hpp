#pragma once
#include <Eigen/Dense>
#include <array>
#include <cstdint>

namespace fusion {

constexpr int kStateDim = 15;  // pos(3)+vel(3)+quat(4)+ab(3)+gb(3)
constexpr int kMeasDim  = 3;
constexpr int kBufSize  = 256;

using StateVec = Eigen::Matrix<double, kStateDim, 1>;
using StateMat = Eigen::Matrix<double, kStateDim, kStateDim>;
using MeasVec  = Eigen::Matrix<double, kMeasDim,  1>;
using MeasMat  = Eigen::Matrix<double, kMeasDim,  kStateDim>;
using KalmanK  = Eigen::Matrix<double, kStateDim, kMeasDim>;

struct Snapshot {
    StateVec     x;
    StateMat     P;
    std::int64_t timestamp_us{0};
    float        accel[3]{};
    float        gyro[3]{};
};

class EKF {
public:
    EKF();

    void predict(const float accel[3], const float gyro[3],
                 std::int64_t timestamp_us);

    void update_vision(const MeasVec& z_pos, std::int64_t timestamp_us);

    const StateVec& state() const noexcept { return x_; }

    // Returns current orientation as quaternion [w,x,y,z]
    Eigen::Vector4d quaternion() const noexcept {
        return x_.segment<4>(6);
    }

private:
    // Skew-symmetric matrix for cross product: a× 
    static Eigen::Matrix3d skew(const Eigen::Vector3d& v) noexcept {
        Eigen::Matrix3d S;
        S <<     0, -v(2),  v(1),
              v(2),     0, -v(0),
             -v(1),  v(0),     0;
        return S;
    }

    // Rotation matrix from quaternion [w,x,y,z]
    static Eigen::Matrix3d R_from_quat(const Eigen::Vector4d& q) noexcept {
        const double w=q(0), x=q(1), y=q(2), z=q(3);
        Eigen::Matrix3d R;
        R << 1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y),
               2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x),
               2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y);
        return R;
    }

    void normalise_quat() noexcept {
        x_.segment<4>(6).normalize();
    }

    void store_snapshot(std::int64_t timestamp_us,
                        const float accel[3], const float gyro[3]);
    int  find_snapshot(std::int64_t timestamp_us) const;
    void repropagate(int from_idx);

    StateVec x_;
    StateMat P_;
    StateMat Q_;
    MeasMat  H_;
    Eigen::Matrix<double, kMeasDim, kMeasDim> R_;

    std::array<Snapshot, kBufSize> buf_;
    int buf_head_{0};
    int buf_count_{0};

    static constexpr double kDt = 1.0 / 200.0;
    static constexpr double kG  = 9.81;
};

} // namespace fusion
