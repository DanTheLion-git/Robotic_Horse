cmake_minimum_required(VERSION 3.8)
project(robotic_horse_description)

find_package(ament_cmake REQUIRED)

# Install everything so launch/urdf/rviz are accessible at runtime
install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
