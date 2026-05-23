#include "fusion/ekf.hpp"
#include <cstring>
#include <cstdio>

namespace fusion {

EKF::EKF() {
    x_.setZero();
    x_(6) = 1.0;  // quaternion w=1 → identity rotation

    P_ = StateMat::Identity() * 0.1;

    Q_.setZero();
    Q_.block<3,3>(0,0)  = Eigen::Matrix3d::Identity() * 1e-6;
    Q_.block<3,3>(3,3)  = Eigen::Matrix3d::Identity() * 1e-4;
    Q_.block<4,4>(6,6)  = Eigen::Matrix4d::Identity() * 1e-6;
    Q_.block<3,3>(10,10)= Eigen::Matrix3d::Identity() * 1e-7;
    Q_.block<2,2>(13,13)= Eigen::Matrix2d::Identity() * 1e-8;

    H_.setZero();
    H_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();

    R_ = Eigen::Matrix<double,kMeasDim,kMeasDim>::Identity() * 0.05;
}

void EKF::predict(const float accel[3], const float gyro[3],
                  std::int64_t timestamp_us) {

    const Eigen::Vector4d q  = x_.segment<4>(6);
    const Eigen::Matrix3d Rq = R_from_quat(q);

    // State indices:
    // 0-2: pos, 3-5: vel, 6-9: quat, 10-12: accel_bias, 13-14: gyro_bias(xy)
    const Eigen::Vector3d a_body{
        static_cast<double>(accel[0]) - x_(10),
        static_cast<double>(accel[1]) - x_(11),
        static_cast<double>(accel[2]) - x_(12)
    };

    const Eigen::Vector3d w_body{
        static_cast<double>(gyro[0]) - x_(13),
        static_cast<double>(gyro[1]) - x_(14),
        static_cast<double>(gyro[2])           // z gyro bias not tracked
    };

    // Rotate to world frame and remove gravity
    const Eigen::Vector3d a_world = Rq * a_body
                                  - Eigen::Vector3d{0.0, 0.0, kG};

    // Position and velocity integration
    x_.segment<3>(0) += x_.segment<3>(3) * kDt
                      + 0.5 * a_world * kDt * kDt;
    x_.segment<3>(3) += a_world * kDt;

    // Quaternion kinematics: q_dot = 0.5 * Omega(w) * q
    const double wx=w_body(0), wy=w_body(1), wz=w_body(2);
    Eigen::Matrix4d Omega;
    Omega <<   0, -wx, -wy, -wz,
              wx,   0,  wz, -wy,
              wy, -wz,   0,  wx,
              wz,  wy, -wx,   0;

    x_.segment<4>(6) += 0.5 * Omega * x_.segment<4>(6) * kDt;
    normalise_quat();

    // Jacobian F
    StateMat F = StateMat::Identity();
    F.block<3,3>(0,3)  = Eigen::Matrix3d::Identity() * kDt;
    F.block<3,3>(3,10) = -Rq * kDt;
    P_ = F * P_ * F.transpose() + Q_;

    store_snapshot(timestamp_us, accel, gyro);
}

void EKF::update_vision(const MeasVec& z_pos, std::int64_t timestamp_us) {
    const int idx = find_snapshot(timestamp_us);

    auto apply_update = [&]() {
        const MeasVec  y = z_pos - H_ * x_;
        const auto     S = H_ * P_ * H_.transpose() + R_;
        const KalmanK  K = P_ * H_.transpose() * S.inverse();
        x_ += K * y;
        const StateMat I_KH = StateMat::Identity() - K * H_;
        P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose();
        normalise_quat();
    };

    if (idx >= 0) {
        x_ = buf_[idx].x;
        P_ = buf_[idx].P;
        apply_update();
        repropagate(idx);
    } else {
        apply_update();
    }

    std::printf(
        "[EKF ] pos=(%.4f,%.4f,%.4f) vel=(%.4f,%.4f,%.4f) "
        "q=(%.3f,%.3f,%.3f,%.3f)\n",
        x_(0),x_(1),x_(2), x_(3),x_(4),x_(5),
        x_(6),x_(7),x_(8),x_(9));
}

void EKF::store_snapshot(std::int64_t timestamp_us,
                         const float accel[3], const float gyro[3]) {
    Snapshot& s    = buf_[buf_head_];
    s.x            = x_;
    s.P            = P_;
    s.timestamp_us = timestamp_us;
    std::memcpy(s.accel, accel, sizeof(s.accel));
    std::memcpy(s.gyro,  gyro,  sizeof(s.gyro));
    buf_head_      = (buf_head_ + 1) & (kBufSize - 1);
    if (buf_count_ < kBufSize) ++buf_count_;
}

int EKF::find_snapshot(std::int64_t timestamp_us) const {
    int best           = -1;
    std::int64_t best_diff = INT64_MAX;
    for (int i = 0; i < buf_count_; ++i) {
        const int idx   = (buf_head_ - 1 - i + kBufSize) & (kBufSize - 1);
        const auto diff = std::abs(buf_[idx].timestamp_us - timestamp_us);
        if (diff < best_diff) { best_diff = diff; best = idx; }
    }
    return best;
}

void EKF::repropagate(int from_idx) {
    int steps = 0;
    int idx   = (from_idx + 1) & (kBufSize - 1);
    while (idx != buf_head_ && steps < kBufSize) {
        predict(buf_[idx].accel, buf_[idx].gyro,
                buf_[idx].timestamp_us);
        idx = (idx + 1) & (kBufSize - 1);
        ++steps;
    }
}

} // namespace fusion
