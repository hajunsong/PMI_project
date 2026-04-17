cmake_minimum_required(VERSION 3.16)
project(robot_arm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose Debug, Release, RelWithDebInfo, or MinSizeRel." FORCE)
endif()

# Eigen: 시스템 패키지 우선, 없으면 FetchContent (헤더 전용)
# 수동 설치: https://eigen.tuxfamily.org/
# Ubuntu: sudo apt install libeigen3-dev
find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
    include(FetchContent)
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG        3.4.0
    )
    FetchContent_MakeAvailable(eigen)
endif()

add_library(${PROJECT_NAME} STATIC
    src/kinematics.cpp
)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

add_executable(kinematics_demo
    apps/kinematics_demo.cpp
)
target_link_libraries(kinematics_demo PRIVATE ${PROJECT_NAME})
