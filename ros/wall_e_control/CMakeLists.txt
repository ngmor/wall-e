cmake_minimum_required(VERSION 3.8)
project(wall_e_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED) # TODO remove when roboclaw messages defined
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(roboclaw_ros QUIET)
find_package(wall_e_interfaces REQUIRED)

# Logic to choose whether or not to use RoboClaw motor controller
# Instead of Arduino motor shield
# TODO document
if(NOT DEFINED DISABLE_ROBOCLAW)
  set(DISABLE_ROBOCLAW OFF)
endif()

if(${roboclaw_ros_FOUND} AND NOT ${DISABLE_ROBOCLAW})
  set(USE_ROBOCLAW ON)
else()
  set(USE_ROBOCLAW OFF)
endif()

if(USE_ROBOCLAW)
  add_compile_definitions(USE_ROBOCLAW)
  set(roboclaw_ros_DEPENDENCY roboclaw_ros)
endif()

# Build

# Kinematics library
file(GLOB_RECURSE WALL_E_KINEMATICS_LIB_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/kinematics/*)
add_library(wall_e_kinematics_lib ${WALL_E_KINEMATICS_LIB_SRC})
target_include_directories(wall_e_kinematics_lib
  PUBLIC include
)

# WALL-E kinematics node
add_executable(wall_e_kinematics src/nodes/wall_e_kinematics.cpp)
target_link_libraries(wall_e_kinematics
  wall_e_kinematics_lib
)
ament_target_dependencies(wall_e_kinematics
  rclcpp
  ${roboclaw_ros_DEPENDENCY}
  std_msgs # TODO remove when roboclaw messages are defined
  geometry_msgs
  wall_e_interfaces
)

# Install

# Python package
ament_python_install_package(wall_e
  PACKAGE_DIR wall_e
)

# Nodes

# WALL-E audio node
install(PROGRAMS
  src/nodes/wall_e_audio.py RENAME wall_e_audio
  DESTINATION lib/${PROJECT_NAME}
)

# WALL-E arduino node
install(PROGRAMS
  src/nodes/wall_e_arduino.py RENAME wall_e_arduino
  DESTINATION lib/${PROJECT_NAME}
)

# WALL-E C++ nodes
install(TARGETS
  wall_e_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

# Config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Sounds
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../web_interface/static/sounds
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
