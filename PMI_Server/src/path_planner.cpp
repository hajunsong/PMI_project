#include "path_planner.h"

#include <pmi_kinematics/pmi_kinematics.hpp>

#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr std::array<double, 3> kRollPitchTarget = {-kPi / 2.0, 0.0, 0.0};
} // namespace

void PathPlanner::setWaypoints(const std::vector<Waypoint> &waypoints)
{
    waypoints_ = waypoints;
    path_.clear();
    pathIndex_ = 0;
    running_ = false;
}

void PathPlanner::setInitialJointRad(const std::array<double, 4> &qInitRad)
{
    q_ = qInitRad;
    fk();
}

std::vector<std::array<double, 3>> PathPlanner::quinticPath(double p0, double pf, double tf, double h)
{
    if (tf <= 0.0 || h <= 0.0)
        return {};
    const int n = std::max(1, static_cast<int>(std::round(tf / h)));
    const double a0 = p0;
    const double a1 = 0.0;
    const double a2 = 0.0;
    const double a3 = 10.0 * (pf - p0) / std::pow(tf, 3);
    const double a4 = -15.0 * (pf - p0) / std::pow(tf, 4);
    const double a5 = 6.0 * (pf - p0) / std::pow(tf, 5);

    std::vector<std::array<double, 3>> out;
    out.reserve(static_cast<std::size_t>(n + 1));
    for (int i = 0; i <= n; ++i) {
        const double t = tf * static_cast<double>(i) / static_cast<double>(n);
        const double t2 = t * t;
        const double t3 = t2 * t;
        const double t4 = t3 * t;
        const double t5 = t4 * t;
        out.push_back({a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5,
            a1 + 2.0 * a2 * t + 3.0 * a3 * t2 + 4.0 * a4 * t3 + 5.0 * a5 * t4,
            2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t2 + 20.0 * a5 * t3});
    }
    return out;
}

bool PathPlanner::plan(double dt)
{
    path_.clear();
    pathIndex_ = 0;
    running_ = false;
    lastPlanClipped_ = false;
    lastPlanFirstClippedIdx_ = 0;
    if (waypoints_.size() < 2 || dt <= 0.0)
        return false;
    for (std::size_t i = 1; i < waypoints_.size(); ++i) {
        const Waypoint &a = waypoints_[i - 1];
        const Waypoint &b = waypoints_[i];
        const double segT = b.t - a.t;
        if (segT <= 0.0)
            return false;
        const auto px = quinticPath(a.x, b.x, segT, dt);
        const auto py = quinticPath(a.y, b.y, segT, dt);
        const auto pz = quinticPath(a.z, b.z, segT, dt);
        const std::size_t n = std::min({px.size(), py.size(), pz.size()});
        if (n == 0)
            return false;
        for (std::size_t k = 0; k < n; ++k) {
            if (i < waypoints_.size() - 1 && k == n - 1)
                continue;
            // quinticPath returns {pos, vel, acc} per sample → keep pos + vel for VSD task-space PD.
            path_.push_back({px[k][0], py[k][0], pz[k][0], px[k][1], py[k][1], pz[k][1]});
        }
    }
    if (path_.empty())
        return false;

    // Pre-IK every sample under joint limits and replace its EE position with what is actually reachable.
    // Orientation (roll = -π/2, pitch = 0) is tracked along with position inside the same DLS — when joints
    // saturate, the active-set clamp in ikSolveTo() keeps orientation as best-effort while the position
    // gets deflected to the closest reachable point. The achieved FK position becomes the new path sample,
    // converged or not.
    const std::array<double, 4> qStart = q_;
    bool anyClipped = false;
    std::size_t firstClippedIdx = 0;
    std::size_t unconvergedCount = 0;
    for (std::size_t k = 0; k < path_.size(); ++k) {
        Sample &s = path_[k];
        const double desiredX = s.x, desiredY = s.y, desiredZ = s.z;
        const bool converged = ikSolveTo(s);
        if (!converged)
            ++unconvergedCount;
        fk();
        const double dxPos = ee_pos_[0] - desiredX;
        const double dyPos = ee_pos_[1] - desiredY;
        const double dzPos = ee_pos_[2] - desiredZ;
        s.x = ee_pos_[0];
        s.y = ee_pos_[1];
        s.z = ee_pos_[2];
        const double clipDist = std::sqrt(dxPos * dxPos + dyPos * dyPos + dzPos * dzPos);
        constexpr double kClipReportThreshold = 1e-3; // 1 mm — log when a sample was deflected
        if (clipDist > kClipReportThreshold) {
            if (!anyClipped) {
                anyClipped = true;
                firstClippedIdx = k;
            }
        }
    }
    // Re-derive workspace velocities from the (possibly modified) positions via central differences.
    const double dtSec = (path_.size() > 1) ? dt : 0.0;
    if (dtSec > 0.0) {
        for (std::size_t k = 0; k < path_.size(); ++k) {
            const std::size_t kp = (k + 1 < path_.size()) ? k + 1 : k;
            const std::size_t km = (k > 0) ? k - 1 : k;
            const double denom = (kp == k || km == k) ? dtSec : (2.0 * dtSec);
            path_[k].vx = (path_[kp].x - path_[km].x) / denom;
            path_[k].vy = (path_[kp].y - path_[km].y) / denom;
            path_[k].vz = (path_[kp].z - path_[km].z) / denom;
        }
    }
    // Restore q_ so step() begins from the actual starting joint state.
    q_ = qStart;
    fk();

    lastPlanClipped_ = anyClipped;
    lastPlanFirstClippedIdx_ = firstClippedIdx;
    if (anyClipped) {
        std::cerr << "[PMI] path clipped to joint limits — first deflected sample at index " << firstClippedIdx
                  << "/" << path_.size()
                  << " (orientation kept at target; EE position pulled inside reachable workspace)\n";
    }
    if (unconvergedCount > 0) {
        std::cerr << "[PMI] note: " << unconvergedCount << "/" << path_.size()
                  << " samples did not fully converge in IK — using best-effort q (path may jitter near limits)\n";
    }
    return true;
}

void PathPlanner::setJointLimits(const std::array<double, 4> &qMinRad, const std::array<double, 4> &qMaxRad)
{
    qMin_ = qMinRad;
    qMax_ = qMaxRad;
}

void PathPlanner::start()
{
    if (!path_.empty()) {
        running_ = true;
        pathIndex_ = 0;
    }
}

void PathPlanner::stop()
{
    running_ = false;
}

double PathPlanner::norm5(const std::array<double, 5> &v)
{
    double s = 0.0;
    for (double x : v)
        s += x * x;
    return std::sqrt(s);
}

double PathPlanner::wrapToPi(double angle)
{
    double wrapped = std::fmod(angle + kPi, 2.0 * kPi);
    if (wrapped < 0.0)
        wrapped += 2.0 * kPi;
    return wrapped - kPi;
}

void PathPlanner::currentEeFromInternalFk(double outXyz[3])
{
    fk();
    outXyz[0] = ee_pos_[0];
    outXyz[1] = ee_pos_[1];
    outXyz[2] = ee_pos_[2];
}

void PathPlanner::fk()
{
    const Eigen::Vector4d q(q_[0], q_[1], q_[2], q_[3]);
    Eigen::Vector3d re;
    Eigen::Vector3d rpy;
    pmi::fk_ee_pose_joint_rad(q, re, rpy);
    ee_pos_[0] = re[0];
    ee_pos_[1] = re[1];
    ee_pos_[2] = re[2];
    ee_roll_ = rpy[0];
    ee_pitch_ = rpy[1];
}

std::array<double, 5> PathPlanner::currentErrorTo(const Sample &s) const
{
    return {s.x - ee_pos_[0], s.y - ee_pos_[1], s.z - ee_pos_[2], wrapToPi(kRollPitchTarget[0] - ee_roll_), wrapToPi(kRollPitchTarget[1] - ee_pitch_)};
}

bool PathPlanner::ikSolveTo(const Sample &s)
{
    // Same IK step as `ControlMain::run_ik` plus active-set joint-limit clamping:
    //   δq = α Jᵣᵀ (Jᵣ Jᵣᵀ + λ²I)⁻¹ e   (Jᵣ keeps only columns of joints not yet at a bound)
    // When a joint hits qMin_/qMax_, it is fixed at the bound and dropped from the active set so the
    // remaining joints continue to reduce the 5-D task error (orientation + position) as best they can.
    constexpr int kMaxIter = 200;
    constexpr double kErrTol = 1e-3;
    constexpr double kDamping = 1e-7;
    constexpr double kAlpha = 0.6;
    constexpr double kBoundEps = 1e-9;

    std::array<bool, 4> clamped{{false, false, false, false}};
    // If the seed q_ is already outside limits, snap it inside so iterations start from a feasible point.
    for (int k = 0; k < 4; ++k) {
        if (q_[static_cast<size_t>(k)] < qMin_[static_cast<size_t>(k)])
            q_[static_cast<size_t>(k)] = qMin_[static_cast<size_t>(k)];
        if (q_[static_cast<size_t>(k)] > qMax_[static_cast<size_t>(k)])
            q_[static_cast<size_t>(k)] = qMax_[static_cast<size_t>(k)];
    }

    for (int iter = 0; iter < kMaxIter; ++iter) {
        fk();
        const auto err = currentErrorTo(s);
        if (norm5(err) < kErrTol)
            return true;

        const Eigen::Vector4d q(q_[0], q_[1], q_[2], q_[3]);
        Eigen::Matrix<double, 5, 4> J;
        pmi::jacobian_5x4_joint_rad(q, J);

        // Zero-out columns of clamped joints so they cannot move.
        for (int k = 0; k < 4; ++k) {
            if (clamped[static_cast<size_t>(k)])
                J.col(k).setZero();
        }

        Eigen::Matrix<double, 5, 1> err_vec;
        err_vec << err[0], err[1], err[2], err[3], err[4];

        Eigen::Matrix<double, 5, 5> JJT_reg = J * J.transpose();
        JJT_reg.diagonal().array() += kDamping * kDamping;

        Eigen::Matrix<double, 5, 1> y = JJT_reg.partialPivLu().solve(err_vec);
        Eigen::Vector4d dq = kAlpha * J.transpose() * y;
        for (int k = 0; k < 4; ++k) {
            if (clamped[static_cast<size_t>(k)])
                dq(k) = 0.0;
        }

        bool newClampThisIter = false;
        for (int k = 0; k < 4; ++k) {
            if (clamped[static_cast<size_t>(k)])
                continue;
            const double qk = q_[static_cast<size_t>(k)] + dq(k);
            const double qmin = qMin_[static_cast<size_t>(k)];
            const double qmax = qMax_[static_cast<size_t>(k)];
            if (qk > qmax + kBoundEps) {
                q_[static_cast<size_t>(k)] = qmax;
                clamped[static_cast<size_t>(k)] = true;
                newClampThisIter = true;
            } else if (qk < qmin - kBoundEps) {
                q_[static_cast<size_t>(k)] = qmin;
                clamped[static_cast<size_t>(k)] = true;
                newClampThisIter = true;
            } else {
                q_[static_cast<size_t>(k)] = qk;
            }
        }

        // If a new clamp activated, restart the iteration with the reduced active set so dq is recomputed
        // for the remaining axes (without this, a single big dq could drag a free axis through saturation).
        if (newClampThisIter)
            continue;
    }
    fk();
    const auto finalErr = currentErrorTo(s);
    return norm5(finalErr) < kErrTol;
}

bool PathPlanner::step(std::array<double, 4> &jointRadOut)
{
    if (!running_ || pathIndex_ >= path_.size())
        return false;
    // After plan() rewrites samples to reachable EE positions, this IK should converge cleanly. Joint
    // limits are still enforced as a safety net (e.g. if the live joint state drifted into the boundary).
    const bool solved = ikSolveTo(path_[pathIndex_]);
    ++pathIndex_;
    if (pathIndex_ >= path_.size())
        running_ = false;
    if (!solved) {
        const std::size_t failedAt = pathIndex_ - 1;
        const Sample &s = path_[failedAt];
        fk();
        std::cerr << "[PMI] trajectory IK did not fully converge at sample " << failedAt << "/" << path_.size()
                  << " target_xyz=(" << s.x << ", " << s.y << ", " << s.z << ") current_ee_xyz=(" << ee_pos_[0]
                  << ", " << ee_pos_[1] << ", " << ee_pos_[2] << ") — using best-effort q\n";
        // Use the best-effort q_ instead of failing hard; samples are already inside reachable workspace.
        jointRadOut = q_;
        return true;
    }
    jointRadOut = q_;
    return true;
}

bool PathPlanner::currentDesiredPose(DesiredPose &out) const
{
    if (!running_ || pathIndex_ >= path_.size())
        return false;
    const Sample &s = path_[pathIndex_];
    out.x = s.x;
    out.y = s.y;
    out.z = s.z;
    out.roll = -kPi / 2.0;
    out.pitch = 0.0;
    out.yaw = 0.0;
    return true;
}

bool PathPlanner::currentDesiredPoseAndVelocity(double outPos[3], double &outRoll, double &outPitch,
                                                double outVel[5]) const
{
    if (!running_ || pathIndex_ >= path_.size())
        return false;
    const Sample &s = path_[pathIndex_];
    outPos[0] = s.x;
    outPos[1] = s.y;
    outPos[2] = s.z;
    outRoll = -kPi / 2.0;
    outPitch = 0.0;
    outVel[0] = s.vx;
    outVel[1] = s.vy;
    outVel[2] = s.vz;
    outVel[3] = 0.0; // constant orientation target → zero angular rate desired
    outVel[4] = 0.0;
    return true;
}
