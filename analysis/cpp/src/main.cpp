#include <pmi/types.hpp>

#include <iostream>

int main() {
    const pmi::Vec3 v(1.0, 2.0, 3.0);
    std::cout << "pmi_cpp (Eigen): ||v||^2 = " << v.squaredNorm() << std::endl;
    return 0;
}
