#ifndef FASTER_LIO_SO3_MATH_H
#define FASTER_LIO_SO3_MATH_H

#include <Eigen/Core>
#include <cmath>

namespace faster_lio {

template <typename T>
inline Eigen::Matrix<T, 3, 3> SKEW_SYM_MATRIX(const Eigen::Matrix<T, 3, 1> &v) {
    Eigen::Matrix<T, 3, 3> m;
    m << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
    return m;
}

template <typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt) {
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    if (ang_vel_norm > 0.0000001) {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix<T, 3, 3> K;

        K = SKEW_SYM_MATRIX(r_axis);

        T r_ang = ang_vel_norm * dt;

        /// Roderigous Tranformation
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    } else {
        return Eye3;
    }
}

}  // namespace faster_lio
#endif