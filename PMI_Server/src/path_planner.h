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
    bool step(std::array<double, 4> &jointRadOut);
    bool currentDesiredPose(DesiredPose &out) const;

private:
    struct Sample {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct BodyState {
        double qi = 0.0;
        std::array<double, 3> ri{};
        std::array<double, 3> hi{};
    };

    static std::vector<std::array<double, 3>> quinticPath(double p0, double pf, double tf, double h);
    static std::array<double, 3> cross(const std::array<double, 3> &a, const std::array<double, 3> &b);
    static std::array<double, 3> sub3(const std::array<double, 3> &a, const std::array<double, 3> &b);
    static double norm5(const std::array<double, 5> &v);
    static double wrapToPi(double angle);
    static bool solveLinear5x5(double a[5][5], double b[5], double x[5]);

    void fk();
    void jacobian(double j[5][4]) const;
    std::array<double, 5> currentErrorTo(const Sample &s) const;
    bool ikSolveTo(const Sample &s);

    std::vector<Waypoint> waypoints_;
    std::vector<Sample> path_;
    std::size_t pathIndex_ = 0;
    bool running_ = false;

    std::array<double, 4> q_{{-2.7367009, 1.0880061, 1.1749032, -0.87868275}};
    std::array<BodyState, 4> body_{};
};

#endif
