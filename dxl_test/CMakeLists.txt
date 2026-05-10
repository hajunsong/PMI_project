cmake_minimum_required(VERSION 3.14)
project(dxl_test VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── 빌드 타입 기본값 ──────────────────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_DEBUG   "-g -O0")

# ── Dynamixel SDK ─────────────────────────────────────────────────────────────
set(DXL_SDK_INCLUDE_DIR /usr/local/include/dynamixel_sdk)
set(DXL_SDK_LIB_DIR     /usr/local/lib)
set(DXL_SDK_LIB         dxl_x64_cpp)

find_library(DXL_LIB
  NAMES ${DXL_SDK_LIB}
  HINTS ${DXL_SDK_LIB_DIR}
  REQUIRED
)

# ── 소스 파일 ─────────────────────────────────────────────────────────────────
file(GLOB_RECURSE SOURCES src/*.cpp)

# ── 실행 파일 ─────────────────────────────────────────────────────────────────
add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${DXL_SDK_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  ${DXL_LIB}
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)

# ── 설치 ─────────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

message(STATUS "Build type  : ${CMAKE_BUILD_TYPE}")
message(STATUS "DXL SDK     : ${DXL_SDK_INCLUDE_DIR}")
message(STATUS "DXL library : ${DXL_LIB}")
