// 독립 실행: Eigen + 정적 라이브러리 링크 확인용.
#include "robot_arm/kinematics.hpp"

#include <iostream>

int main()
{
    Eigen::Matrix2d m;
    m << 1, 2, 3, 4;
    std::cout << "Eigen ok, det=" << m.determinant() << "\n";

    robot_arm::Kinematics4Axis kin;
    const robot_arm::JointVector4 q = robot_arm::JointVector4::Ones() * 0.1;
    const robot_arm::PoseVector p = kin.forward(q);
    std::cout << "forward(stub) pose norm=" << p.norm() << "\n";
    return 0;
}
