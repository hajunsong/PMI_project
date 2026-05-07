#include "path_planner.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr std::array<std::array<double, 3>, 4> kSijP = {{{{0.0, 0.0, -0.22}}, {{0.0, -0.23, 0.0}}, {{0.23, 0.0, 0.0}}, {{0.18, 0.0, 0.0}}}};
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
            path_.push_back({px[k][0], py[k][0], pz[k][0]});
        }
    }
    return !path_.empty();
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

std::array<double, 3> PathPlanner::cross(const std::array<double, 3> &a, const std::array<double, 3> &b)
{
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

std::array<double, 3> PathPlanner::sub3(const std::array<double, 3> &a, const std::array<double, 3> &b)
{
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
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

bool PathPlanner::solveLinear5x5(double a[5][5], double b[5], double x[5])
{
    for (int i = 0; i < 5; ++i) {
        int pivot = i;
        double maxAbs = std::fabs(a[i][i]);
        for (int r = i + 1; r < 5; ++r) {
            const double v = std::fabs(a[r][i]);
            if (v > maxAbs) {
                maxAbs = v;
                pivot = r;
            }
        }
        if (maxAbs < 1e-12)
            return false;
        if (pivot != i) {
            for (int c = i; c < 5; ++c)
                std::swap(a[i][c], a[pivot][c]);
            std::swap(b[i], b[pivot]);
        }
        const double diag = a[i][i];
        for (int c = i; c < 5; ++c)
            a[i][c] /= diag;
        b[i] /= diag;
        for (int r = 0; r < 5; ++r) {
            if (r == i)
                continue;
            const double f = a[r][i];
            for (int c = i; c < 5; ++c)
                a[r][c] -= f * a[i][c];
            b[r] -= f * b[i];
        }
    }
    for (int i = 0; i < 5; ++i)
        x[i] = b[i];
    return true;
}

void PathPlanner::fk()
{
    for (int i = 0; i < 4; ++i)
        body_[i].qi = q_[static_cast<std::size_t>(i)];

    body_[0].ri = kSijP[0];
    body_[0].hi = {0.0, 0.0, 1.0};
    body_[1].ri = {body_[0].ri[0], body_[0].ri[1] + kSijP[1][1], body_[0].ri[2]};
    body_[1].hi = {0.0, 1.0, 0.0};
    body_[2].ri = {body_[1].ri[0] + kSijP[2][0], body_[1].ri[1], body_[1].ri[2]};
    body_[2].hi = {1.0, 0.0, 0.0};
    body_[3].ri = {body_[2].ri[0] + kSijP[3][0], body_[2].ri[1], body_[2].ri[2]};
    body_[3].hi = {1.0, 0.0, 0.0};
}

void PathPlanner::jacobian(double j[5][4]) const
{
    const std::array<double, 3> ee = body_[3].ri;
    for (int i = 0; i < 4; ++i) {
        const auto c = cross(body_[i].hi, sub3(ee, body_[i].ri));
        j[0][i] = c[0];
        j[1][i] = c[1];
        j[2][i] = c[2];
        j[3][i] = 0.0;
        j[4][i] = 0.0;
    }
    j[3][0] = 1.0;
    j[4][1] = 1.0;
}

std::array<double, 5> PathPlanner::currentErrorTo(const Sample &s) const
{
    return {s.x - body_[3].ri[0], s.y - body_[3].ri[1], s.z - body_[3].ri[2], wrapToPi(kRollPitchTarget[0] - q_[0]), wrapToPi(kRollPitchTarget[1] - q_[1])};
}

bool PathPlanner::ikSolveTo(const Sample &s)
{
    constexpr int kMaxIter = 100;
    constexpr double kErrTol = 1e-3;
    constexpr double kDamping = 1e-4;
    constexpr double kAlpha = 0.6;
    for (int iter = 0; iter < kMaxIter; ++iter) {
        fk();
        const auto err = currentErrorTo(s);
        if (norm5(err) < kErrTol)
            return true;

        double j[5][4]{};
        jacobian(j);
        double a[5][5]{};
        double b[5]{};
        for (int r = 0; r < 5; ++r) {
            b[r] = err[static_cast<std::size_t>(r)];
            for (int c = 0; c < 5; ++c) {
                double v = 0.0;
                for (int k = 0; k < 4; ++k)
                    v += j[r][k] * j[c][k];
                a[r][c] = v + (r == c ? (kDamping * kDamping) : 0.0);
            }
        }
        double y[5]{};
        if (!solveLinear5x5(a, b, y))
            return false;
        for (int k = 0; k < 4; ++k) {
            double dq = 0.0;
            for (int r = 0; r < 5; ++r)
                dq += j[r][k] * y[r];
            q_[static_cast<std::size_t>(k)] += kAlpha * dq;
        }
    }
    return false;
}

bool PathPlanner::step(std::array<double, 4> &jointRadOut)
{
    if (!running_ || pathIndex_ >= path_.size())
        return false;
    const bool solved = ikSolveTo(path_[pathIndex_]);
    ++pathIndex_;
    if (pathIndex_ >= path_.size())
        running_ = false;
    if (!solved)
        return false;
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
