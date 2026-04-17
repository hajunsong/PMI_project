cmake_minimum_required(VERSION 3.16)
# RS485 테스트 타깃 엔코더: Same Sky AMT212H-V (14-bit multi-turn, 가변 baud)
project(amt21_rs485 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose Debug, Release, RelWithDebInfo, or MinSizeRel." FORCE)
endif()

add_library(amt21 STATIC
    src/rs485_port.cpp
    src/amt21_driver.cpp
    src/protocol.cpp
)
target_include_directories(amt21 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(amt21_cli apps/amt21_cli.cpp)
target_link_libraries(amt21_cli PRIVATE amt21)
