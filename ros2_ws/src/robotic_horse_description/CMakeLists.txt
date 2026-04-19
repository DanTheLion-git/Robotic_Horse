cmake_minimum_required(VERSION 3.8)
project(robotic_horse_description)

find_package(ament_cmake REQUIRED)

# Install URDF files so they are accessible at runtime
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
