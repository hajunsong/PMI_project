#include "robot_arm/kinematics.hpp"

namespace robot_arm {

PoseVector Kinematics4Axis::forward(const JointVector4 &q) const
{
    (void)q;
    return PoseVector::Zero();
}

bool Kinematics4Axis::inverse(const PoseVector &target, JointVector4 &qOut) const
{
    (void)target;
    qOut.setZero();
    return false;
}

} // namespace robot_arm
