#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include <array>
#include <cstddef>

namespace pmi {

constexpr std::size_t kNumJoints = 4;

/// joint_deg = motor_deg * kMotorToJointGear[i] (same as ControlMain::read_data `body[i].gear`).
constexpr double kMotorToJointGear[kNumJoints] = {
    32.0 / 60.0, 54.0 / 360.0, 108.0 / 360.0, 108.0 / 360.0};

void joint_rad_from_motor_deg(const double motor_deg[kNumJoints], double q_joint_rad[kNumJoints]);

inline void joint_rad_from_motor_deg(const std::array<double, kNumJoints> &motor_deg, std::array<double, kNumJoints> &q_joint_rad)
{
    joint_rad_from_motor_deg(motor_deg.data(), q_joint_rad.data());
}

/**
 * Forward kinematics matching `ControlMain::read_data` link constants and
 * `ControlMain::position_calculation` propagation (joint angles in rad).
 */
void forward_kinematics_chain(const Eigen::Vector4d &q_joint_rad,
    Eigen::Matrix3d Ai_out[kNumJoints],
    Eigen::Vector3d ri_out[kNumJoints],
    Eigen::Vector3d Hi_out[kNumJoints],
    Eigen::Vector3d sij_out[kNumJoints],
    Eigen::Vector3d &ee_position,
    Eigen::Matrix3d &ee_orientation_Ae);

void fk_ee_pose_joint_rad(const Eigen::Vector4d &q_joint_rad, Eigen::Vector3d &ee_position, Eigen::Vector3d &ee_rpy);

void fk_ee_pose_joint_rad(const Eigen::Vector4d &q_joint_rad, Eigen::Vector3d &ee_position, Eigen::Matrix3d &ee_orientation_Ae);

/** Task Jacobian (5×4): xyz translation + roll/pitch rows — same construction as `ControlMain::jacobian_calculation`. */
void jacobian_5x4_joint_rad(const Eigen::Vector4d &q_joint_rad, Eigen::Matrix<double, 5, 4> &J_out);

/**
 * Joint-space gravity torque (4×1) at posture `q_joint_rad`, using same body masses / COM offsets
 * as `ControlMain::read_data` and the same numerical-gradient method as `ControlMain::joint_gravity_torque`.
 *
 * `g_z` is the world-z component of gravity acceleration (default −9.80665).
 */
Eigen::Vector4d joint_gravity_torque(const Eigen::Vector4d &q_joint_rad, double g_z = -9.80665);

} // namespace pmi
