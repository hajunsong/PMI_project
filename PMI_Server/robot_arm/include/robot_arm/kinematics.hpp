#ifndef ROBOT_ARM_KINEMATICS_HPP
#define ROBOT_ARM_KINEMATICS_HPP

// 4축 로봇팔 기구학 / 역기구학 (Eigen 사용). 구현은 단계적으로 채웁니다.

#include <Eigen/Dense>

namespace robot_arm {

/// 4축 관절 각(rad) 등 상태 벡터 (크기는 모델에 맞게 조정 예정).
using JointVector4 = Eigen::Vector4d;

/// 엔드이펙스 자세 등 (추후 DH에 맞게 확장).
using PoseVector = Eigen::Vector<double, 6>;

class Kinematics4Axis {
public:
    Kinematics4Axis() = default;

    /// 순기구학: 관절각 → EE pose (스텁).
    PoseVector forward(const JointVector4 &q) const;

    /// 역기구학: 목표 pose → 관절각 (스텁; 미해/다해 시 반환 규약은 추후 정의).
    bool inverse(const PoseVector &target, JointVector4 &qOut) const;
};

} // namespace robot_arm

#endif
