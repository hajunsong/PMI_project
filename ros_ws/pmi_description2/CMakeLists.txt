cmake_minimum_required(VERSION 3.8)
project(pmi_description2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf config meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
