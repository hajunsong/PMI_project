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
