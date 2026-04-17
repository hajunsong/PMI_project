cmake_minimum_required(VERSION 3.16)
project(PMI_Client LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose Debug, Release, RelWithDebInfo, or MinSizeRel." FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(PMI_PROTOCOL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pmi_protocol")

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/mainwindow.cpp
    src/mainwindow.h
    src/mainwindow.ui
    src/tcp_client.cpp
    src/tcp_client.h
    ${PMI_PROTOCOL_DIR}/pmi_protocol.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE Qt5::Widgets)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${PMI_PROTOCOL_DIR}
)
