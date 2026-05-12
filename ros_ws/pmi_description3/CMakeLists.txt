cmake_minimum_required(VERSION 3.8)
project(pmi_description3)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf config meshes textures
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
