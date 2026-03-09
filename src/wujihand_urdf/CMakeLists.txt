cmake_minimum_required(VERSION 3.8)
project(wujihand-urdf)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Create urdf directory and install files with correct names
install(
  FILES
    wujihand_left.urdf
  DESTINATION share/${PROJECT_NAME}/urdf
  RENAME left.urdf
)

install(
  FILES
    wujihand_right.urdf
  DESTINATION share/${PROJECT_NAME}/urdf
  RENAME right.urdf
)

ament_package()

