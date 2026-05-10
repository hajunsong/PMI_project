#include <pmi_kinematics/pmi_kinematics.hpp>

#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace pmi {
namespace {

Eigen::Matrix3d euler_zxz(double phi, double theta, double psi)
{
    const double cphi = std::cos(phi);
    const double sphi = std::sin(phi);
    const double cth = std::cos(theta);
    const double sth = std::sin(theta);
    const double cpsi = std::cos(psi);
    const double spsi = std::sin(psi);

    Eigen::Matrix3d rot_phi;
    rot_phi << cphi, -sphi, 0.0, sphi, cphi, 0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix3d rot_theta;
    rot_theta << 1.0, 0.0, 0.0, 0.0, cth, -sth, 0.0, sth, cth;
    Eigen::Matrix3d rot_psi;
    rot_psi << cpsi, -spsi, 0.0, spsi, cpsi, 0.0, 0.0, 0.0, 1.0;
    return rot_phi * rot_theta * rot_psi;
}

Eigen::Matrix3d rot_z(double q)
{
    const double c = std::cos(q);
    const double s = std::sin(q);
    Eigen::Matrix3d r;
    r << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
    return r;
}

Eigen::Vector3d mat2rpy(const Eigen::Matrix3d &A)
{
    const double roll = std::atan2(A(2, 1), A(2, 2));
    const double pitch = std::atan2(-A(2, 0), std::sqrt(A(0, 0) * A(0, 0) + A(1, 0) * A(1, 0)));
    const double yaw = std::atan2(A(1, 0), A(0, 0));
    return {roll, pitch, yaw};
}

/// ∂Rz(q)/∂q — same layout as `ControlMain::jacobian_calculation` (`A01pp_q1`, …).
Eigen::Matrix3d rot_z_deriv(double q)
{
    const double s = std::sin(q);
    const double c = std::cos(q);
    Eigen::Matrix3d m;
    m << -s, -c, 0.0, c, -s, 0.0, 0.0, 0.0, 0.0;
    return m;
}

Eigen::Matrix3d droll_dA(const Eigen::Matrix3d &A, double eps = 1e-15)
{
    const double y = A(2, 1);
    const double x = A(2, 2);
    const double den = y * y + x * x + eps;
    Eigen::Matrix3d G = Eigen::Matrix3d::Zero();
    G(2, 1) = x / den;
    G(2, 2) = -y / den;
    return G;
}

Eigen::Matrix3d dpitch_dA(const Eigen::Matrix3d &A, double eps = 1e-15)
{
    const double y = -A(2, 0);
    const double c = std::sqrt(A(0, 0) * A(0, 0) + A(1, 0) * A(1, 0) + eps);
    const double den = y * y + c * c + eps;
    Eigen::Matrix3d G = Eigen::Matrix3d::Zero();
    G(2, 0) = -c / den;
    if (c > eps) {
        const double fac = -y / den / c;
        G(0, 0) = fac * A(0, 0);
        G(1, 0) = fac * A(1, 0);
    }
    return G;
}

Eigen::Matrix<double, 2, 4> roll_pitch_jacobian_wrt_q(const Eigen::Matrix3d &A, const std::array<Eigen::Matrix3d, 4> &dA_dq, double eps = 1e-5)
{
    const Eigen::Matrix3d Gr = droll_dA(A, eps);
    const Eigen::Matrix3d Gp = dpitch_dA(A, eps);
    Eigen::Matrix<double, 2, 4> J;
    for (int k = 0; k < 4; ++k) {
        J(0, k) = (Gr.array() * dA_dq[static_cast<size_t>(k)].array()).sum();
        J(1, k) = (Gp.array() * dA_dq[static_cast<size_t>(k)].array()).sum();
    }
    return J;
}

} // namespace

void joint_rad_from_motor_deg(const double motor_deg[kNumJoints], double q_joint_rad[kNumJoints])
{
    for (std::size_t i = 0; i < kNumJoints; ++i)
        q_joint_rad[i] = motor_deg[i] * kMotorToJointGear[i] * (M_PI / 180.0);
}

void forward_kinematics_chain(const Eigen::Vector4d &q_joint_rad,
    Eigen::Matrix3d Ai_out[kNumJoints],
    Eigen::Vector3d ri_out[kNumJoints],
    Eigen::Vector3d Hi_out[kNumJoints],
    Eigen::Vector3d sij_out[kNumJoints],
    Eigen::Vector3d &ee_position,
    Eigen::Matrix3d &ee_orientation_Ae)
{
    // ControlMain::read_data — kinematic columns only (same sijp / Cij / sep / Ce order).
    const Eigen::Vector3d sijp[4] = {
        Eigen::Vector3d::Zero(),
        {0.0, 0.0, -0.22},
        {0.0, -0.23, 0.0},
        {0.23, 0.0, 0.0},
    };
    const Eigen::Matrix3d Cij[4] = {
        euler_zxz(0.0, M_PI, 0.0),
        euler_zxz(0.0, M_PI_2, 0.0),
        euler_zxz(-M_PI_2, 0.0, 0.0),
        euler_zxz(0.0, M_PI, 0.0),
    };
    const Eigen::Vector3d sep{0.18, 0.0, 0.0};
    const Eigen::Matrix3d Ce = euler_zxz(-M_PI_2, 0.0, 0.0);
    const Eigen::Vector3d u_vec{0.0, 0.0, 1.0};

    Eigen::Matrix3d prevAi = Eigen::Matrix3d::Identity();
    Eigen::Vector3d prevRi = Eigen::Vector3d::Zero();

    for (int i = 0; i < 4; ++i) {
        const Eigen::Matrix3d Aijpp = rot_z(q_joint_rad(static_cast<Eigen::Index>(i)));
        Ai_out[i] = prevAi * Cij[i] * Aijpp;
        sij_out[i] = prevAi * sijp[i];
        ri_out[i] = prevRi + sij_out[i];
        Hi_out[i] = prevAi * Cij[i] * u_vec;
        prevAi = Ai_out[i];
        prevRi = ri_out[i];
    }

    const Eigen::Vector3d se = Ai_out[3] * sep;
    ee_position = ri_out[3] + se;
    ee_orientation_Ae = Ai_out[3] * Ce;
}

void fk_ee_pose_joint_rad(const Eigen::Vector4d &q_joint_rad, Eigen::Vector3d &ee_position, Eigen::Matrix3d &ee_orientation_Ae)
{
    Eigen::Matrix3d Ai[4];
    Eigen::Vector3d ri[4];
    Eigen::Vector3d Hi[4];
    Eigen::Vector3d sij[4];
    forward_kinematics_chain(q_joint_rad, Ai, ri, Hi, sij, ee_position, ee_orientation_Ae);
}

void fk_ee_pose_joint_rad(const Eigen::Vector4d &q_joint_rad, Eigen::Vector3d &ee_position, Eigen::Vector3d &ee_rpy)
{
    Eigen::Matrix3d Ae;
    fk_ee_pose_joint_rad(q_joint_rad, ee_position, Ae);
    ee_rpy = mat2rpy(Ae);
}

void jacobian_5x4_joint_rad(const Eigen::Vector4d &q_joint_rad, Eigen::Matrix<double, 5, 4> &J_out)
{
    const Eigen::Matrix3d Cij[4] = {
        euler_zxz(0.0, M_PI, 0.0),
        euler_zxz(0.0, M_PI_2, 0.0),
        euler_zxz(-M_PI_2, 0.0, 0.0),
        euler_zxz(0.0, M_PI, 0.0),
    };
    const Eigen::Matrix3d Ce = euler_zxz(-M_PI_2, 0.0, 0.0);

    Eigen::Matrix3d Ai[4];
    Eigen::Vector3d ri[4];
    Eigen::Vector3d Hi[4];
    Eigen::Vector3d sij[4];
    Eigen::Vector3d re;
    Eigen::Matrix3d Ae;
    forward_kinematics_chain(q_joint_rad, Ai, ri, Hi, sij, re, Ae);

    Eigen::Matrix3d Rz[4];
    Eigen::Matrix3d Rzdot[4];
    for (int i = 0; i < 4; ++i) {
        Rz[i] = rot_z(q_joint_rad(i));
        Rzdot[i] = rot_z_deriv(q_joint_rad(i));
    }

    const Eigen::Matrix3d A1_q1 = Cij[0] * Rzdot[0];
    const Eigen::Matrix3d A2_q1 = A1_q1 * Cij[1] * Rz[1];
    const Eigen::Matrix3d A3_q1 = A2_q1 * Cij[2] * Rz[2];
    const Eigen::Matrix3d A4_q1 = A3_q1 * Cij[3] * Rz[3];

    const Eigen::Matrix3d A2_q2 = Ai[0] * Cij[1] * Rzdot[1];
    const Eigen::Matrix3d A3_q2 = A2_q2 * Cij[2] * Rz[2];
    const Eigen::Matrix3d A4_q2 = A3_q2 * Cij[3] * Rz[3];

    const Eigen::Matrix3d A3_q3 = Ai[1] * Cij[2] * Rzdot[2];
    const Eigen::Matrix3d A4_q3 = A3_q3 * Cij[3] * Rz[3];

    const Eigen::Matrix3d A4_q4 = Ai[2] * Cij[3] * Rzdot[3];

    const Eigen::Matrix3d Ae_q1 = A4_q1 * Ce;
    const Eigen::Matrix3d Ae_q2 = A4_q2 * Ce;
    const Eigen::Matrix3d Ae_q3 = A4_q3 * Ce;
    const Eigen::Matrix3d Ae_q4 = A4_q4 * Ce;

    Eigen::Matrix<double, 3, 4> jac_pos;
    for (int k = 0; k < 4; ++k)
        jac_pos.col(k) = Hi[k].cross(re - ri[k]);

    const std::array<Eigen::Matrix3d, 4> dAe_dq{Ae_q1, Ae_q2, Ae_q3, Ae_q4};
    const Eigen::Matrix<double, 2, 4> jac_rp = roll_pitch_jacobian_wrt_q(Ae, dAe_dq);

    J_out.topRows<3>() = jac_pos;
    J_out.bottomRows<2>() = jac_rp;
}

namespace {

// ControlMain::read_data masses (kg) and link-frame COM offsets (m), in the same order as joints 1..4.
constexpr double kBodyMass[kNumJoints] = {
    10.0123496865811,
    10.4391437674567,
    10.3406497234359,
    7.01416597186014,
};

const Eigen::Vector3d &bodyRhoIp(int i)
{
    static const Eigen::Vector3d table[kNumJoints] = {
        {0.0026336, -1.68446e-05, -0.111117},
        {6.90559e-05, -0.0851548, -0.00686211},
        {0.0969069, -3.20036e-05, -0.00548574},
        {0.0675884, 0.00443192, 0.000679202},
    };
    return table[static_cast<std::size_t>(i)];
}

double gravity_potential_energy_at(const Eigen::Vector4d &q, double g_z)
{
    Eigen::Matrix3d Ai[kNumJoints];
    Eigen::Vector3d ri[kNumJoints];
    Eigen::Vector3d Hi[kNumJoints];
    Eigen::Vector3d sij[kNumJoints];
    Eigen::Vector3d re;
    Eigen::Matrix3d Ae;
    forward_kinematics_chain(q, Ai, ri, Hi, sij, re, Ae);

    double U = 0.0;
    for (int i = 0; i < static_cast<int>(kNumJoints); ++i) {
        const Eigen::Vector3d ric = ri[i] + Ai[i] * bodyRhoIp(i);
        U += kBodyMass[i] * (-g_z) * ric.z();
    }
    return U;
}

} // namespace

Eigen::Vector4d joint_gravity_torque(const Eigen::Vector4d &q_joint_rad, double g_z)
{
    constexpr double eps = 1e-5;
    Eigen::Vector4d tau_g = Eigen::Vector4d::Zero();
    Eigen::Vector4d q = q_joint_rad;
    for (int j = 0; j < static_cast<int>(kNumJoints); ++j) {
        const double saved = q(j);
        q(j) = saved + eps;
        const double up = gravity_potential_energy_at(q, g_z);
        q(j) = saved - eps;
        const double um = gravity_potential_energy_at(q, g_z);
        q(j) = saved;
        tau_g(j) = (up - um) / (2.0 * eps);
    }
    return tau_g;
}

} // namespace pmi
