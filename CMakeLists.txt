cmake_minimum_required(VERSION 3.8)
project(two_wheel_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install all share directories so package://two_wheel_robot/... resolves.
install(
  DIRECTORY
    description
    meshes
    launch
    config
    worlds
    rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install the Python controller as an executable inside the package.
install(
  PROGRAMS
    scripts/balance_controller.py
    scripts/kinematic_teleop_driver.py
    scripts/snap_upright.py
    scripts/odom_publisher.py
    scripts/square_path.py
    scripts/road_drive.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
