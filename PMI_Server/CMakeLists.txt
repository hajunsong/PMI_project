cmake_minimum_required(VERSION 3.16)
project(PMI_Server LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose Debug, Release, RelWithDebInfo, or MinSizeRel." FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

set(PMI_PROTOCOL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pmi_protocol")

add_executable(${PROJECT_NAME}
    src/main.cpp
    ${PMI_PROTOCOL_DIR}/pmi_protocol.cpp
    src/dxl_protocol2.cpp
    src/tcp_server.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${PMI_PROTOCOL_DIR}
    /usr/local/include
)

target_link_directories(${PROJECT_NAME} PRIVATE /usr/local/lib)

target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads dxl_x64_cpp)
