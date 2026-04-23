cmake_minimum_required(VERSION 3.16)

project(pmi_cpp
    LANGUAGES CXX
    VERSION 0.1.0
    DESCRIPTION "PMI C++ (Eigen) port"
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Ubuntu: sudo apt install libeigen3-dev
# 다른 환경에서 EigenConfig.cmake 경로가 다르면 CMAKE_PREFIX_PATH 설정
find_package(Eigen3 REQUIRED CONFIG)

add_executable(pmi_cpp
    src/main.cpp
)

target_link_libraries(pmi_cpp PRIVATE Eigen3::Eigen)

target_include_directories(pmi_cpp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
