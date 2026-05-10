cmake_minimum_required(VERSION 3.16)
project(pmi_kinematics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED CONFIG)

add_library(pmi_kinematics STATIC
    src/pmi_kinematics.cpp
)

target_include_directories(pmi_kinematics PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(pmi_kinematics PUBLIC Eigen3::Eigen)
