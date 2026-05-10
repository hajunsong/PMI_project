#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

class PathPlanner {
public:
    struct DesiredPose {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
    };

    struct Waypoint {
        double t = 0.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    void setWaypoints(const std::vector<Waypoint> &waypoints);
    void setInitialJointRad(const std::array<double, 4> &qInitRad);
    bool plan(double dt);
    void start();
    void stop();
    bool isRunning() const { return running_; }
    bool hasPlannedPath() const { return !path_.empty(); }
    /// Discrete workspace samples after plan() (same as path_.size()).
    std::size_t pathSampleCount() const { return path_.size(); }
    /// End-effector position (m) from planner FK at current q_; same frame as waypoint x,y,z.
    void currentEeFromInternalFk(double outXyz[3]);
    bool step(std::array<double, 4> &jointRadOut);
    bool currentDesiredPose(DesiredPose &out) const;

    /// Hard joint limits (rad) used by IK active-set clamping inside `plan()` and `ikSolveTo()`.
    /// Defaults match the kinematic limits decided in pmi_description.urdf:
    ///   jnt1 ∈ [-π, π], jnt2 ∈ [-π/2, π/2], jnt3 ∈ [-π/2, π/2], jnt4 ∈ [-π/2, π/2].
    void setJointLimits(const std::array<double, 4> &qMinRad, const std::array<double, 4> &qMaxRad);

    /// True if any sample produced by the last `plan()` had to deflect the desired EE position
    /// because IK saturated against joint limits (orientation kept at the fixed target).
    bool lastPlanWasClipped() const { return lastPlanClipped_; }
    /// Index of the first clipped sample in `plan()` output (only valid when `lastPlanWasClipped()` is true).
    std::size_t lastPlanFirstClippedIndex() const { return lastPlanFirstClippedIdx_; }

    /// Desired EE workspace pose (xyz + roll/pitch) and velocity for the sample currently being executed.
    /// Returns false unless `running_` and a planned path is available.
    /// `outVel = {vx, vy, vz, vroll, vpitch}` (orientation rates are 0 because the current target is constant).
    bool currentDesiredPoseAndVelocity(double outPos[3], double &outRoll, double &outPitch,
                                       double outVel[5]) const;

private:
    struct Sample {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double vx = 0.0;
        double vy = 0.0;
        double vz = 0.0;
    };

    static std::vector<std::array<double, 3>> quinticPath(double p0, double pf, double tf, double h);
    static double norm5(const std::array<double, 5> &v);
    static double wrapToPi(double angle);
    void fk();
    std::array<double, 5> currentErrorTo(const Sample &s) const;
    bool ikSolveTo(const Sample &s);

    std::vector<Waypoint> waypoints_;
    std::vector<Sample> path_;
    std::size_t pathIndex_ = 0;
    bool running_ = false;

    std::array<double, 4> q_{{-2.7367009, 1.0880061, 1.1749032, -0.87868275}};
    static constexpr double kHalfPi = 1.57079632679489661923;
    static constexpr double kPiConst = 3.14159265358979323846;
    std::array<double, 4> qMin_{{-kPiConst, -kHalfPi, -kHalfPi, -kHalfPi}};
    std::array<double, 4> qMax_{{ kPiConst,  kHalfPi,  kHalfPi,  kHalfPi}};
    bool lastPlanClipped_ = false;
    std::size_t lastPlanFirstClippedIdx_ = 0;
    /// Cached EE pose from fk(); updated whenever fk() runs (same model as server_logger / analysis/cpp).
    mutable double ee_pos_[3]{};
    mutable double ee_roll_ = 0.0;
    mutable double ee_pitch_ = 0.0;
};

#endif
