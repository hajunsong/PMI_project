cmake_minimum_required(VERSION 3.16)

project(pmi_cpp
    LANGUAGES CXX
    VERSION 0.1.0
    DESCRIPTION "PMI C++ (Eigen) port"
)

# clangd / IntelliSense용 compile_commands.json (단일 구성 생성기에서 빌드 디렉터리에 생성)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Write compile_commands.json" FORCE)

# 기본 빌드 타입: Release
# - Ninja / Unix Makefiles: ``CMAKE_BUILD_TYPE`` 만 유효. ``CMAKE_DEFAULT_BUILD_TYPE`` 는 지원 안 함.
# - Visual Studio / Xcode 등 다중 구성: ``CMAKE_DEFAULT_BUILD_TYPE`` 로 IDE 기본 구성 지정.
if(CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_DEFAULT_BUILD_TYPE Release CACHE STRING "Multi-config generator default (e.g. VS)" FORCE)
endif()
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type (Debug|Release|RelWithDebInfo|MinSizeRel)" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Ubuntu: sudo apt install libeigen3-dev
# 다른 환경에서 EigenConfig.cmake 경로가 다르면 CMAKE_PREFIX_PATH 설정
find_package(Eigen3 REQUIRED CONFIG)

add_executable(pmi_cpp
    src/main.cpp
    src/controlmain.cpp
)

target_link_libraries(pmi_cpp PRIVATE Eigen3::Eigen)

target_include_directories(pmi_cpp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
