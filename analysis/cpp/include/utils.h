#pragma once

#include <cmath>

#include "types.h"

inline mat3 skew(vec3 x)
{
    return mat3{{0, -x[2], x[1]}, {x[2], 0, -x[0]}, {-x[1], x[0], 0}};
}

inline mat3 euler_zxz(scalar phi, scalar theta, scalar psi)
{
    const scalar cphi = std::cos(phi);
    const scalar sphi = std::sin(phi);
    const scalar cth = std::cos(theta);
    const scalar sth = std::sin(theta);
    const scalar cpsi = std::cos(psi);
    const scalar spsi = std::sin(psi);

    const mat3 rot_phi{{cphi, -sphi, 0}, {sphi, cphi, 0}, {0, 0, 1}};
    const mat3 rot_theta{{1, 0, 0}, {0, cth, -sth}, {0, sth, cth}};
    const mat3 rot_psi{{cpsi, -spsi, 0}, {spsi, cpsi, 0}, {0, 0, 1}};
    return rot_phi * rot_theta * rot_psi;
}

inline vec3 mat2rpy(mat3 A)
{
    const scalar roll = std::atan2(A(2, 1), A(2, 2));
    const scalar pitch = std::atan2(-A(2, 0), std::sqrt(A(0, 0) * A(0, 0) + A(1, 0) * A(1, 0)));
    const scalar yaw = std::atan2(A(1, 0), A(0, 0));
    return vec3{roll, pitch, yaw};
}

inline mat3 droll_dA(const mat3& A, scalar eps = 1e-15){
    const double y = A(2,1);
    const double x = A(2,2);
     
    const double den = y*y + x*x + eps;

    mat3 G = mat3::Zero();

    G(2,1) = x/den;
    G(2,2) = -y/den;

    return G;
}

inline mat3 dpitch_dA(const mat3& A, double eps = 1e-15){
    const double y = -A(2,0);
    const double c = std::sqrt(A(0,0)*A(0,0) + A(1,0)*A(1,0) + eps);

    const double den = y*y + c*c + eps;

    mat3 G = mat3::Zero();

    G(2,0) = -c/den;

    if(c > eps){
        const double fac = -y/den/c;

        G(0,0) = fac*A(0,0);
        G(1,0) = fac*A(1,0);
    }

    return G;
}

inline Eigen::Matrix<double, 2, 4> roll_pitch_jacobian_wrt_q(const mat3& A, const std::array<Eigen::Matrix3d, 4> &dA_dq, double eps = 1e-5){
    const int nq = static_cast<int>(dA_dq.size());

    if (nq == 0){
        throw std::invalid_argument("dA_dq must not be empty");
    }

    mat3 Gr = droll_dA(A, eps);
    mat3 Gp = dpitch_dA(A, eps);

    Eigen::Matrix<double, 2, 4> J;

    for(int k = 0; k < nq; k++){
        const mat3& dAk = dA_dq[k];

        J(0, k) = (Gr.array()*dAk.array()).sum();
        J(1, k) = (Gp.array()*dAk.array()).sum();
    }

    return J;
}