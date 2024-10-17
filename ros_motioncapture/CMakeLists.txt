cmake_minimum_required(VERSION 3.8)
project(ros_motioncapture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INSTALL_RPATH "$ORIGIN")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

# add dependencies
set(LIBMOTIONCAPTURE_BUILD_PYTHON_BINDINGS OFF)
set(LIBMOTIONCAPTURE_BUILD_EXAMPLE OFF)

set(LIBMOTIONCAPTURE_ENABLE_QUALISYS ON)
set(LIBMOTIONCAPTURE_ENABLE_OPTITRACK ON)
set(LIBMOTIONCAPTURE_ENABLE_OPTITRACK_CLOSED_SOURCE ON)
set(LIBMOTIONCAPTURE_ENABLE_VICON ON)
set(LIBMOTIONCAPTURE_ENABLE_VRPN ON)
set(LIBMOTIONCAPTURE_ENABLE_FZMOTION ON)

# To use NOKOV, place their SDK in
# motion_capture_tracking/deps/libmotioncapture/deps/nokov_sdk
# and set this variable to "ON"
set(LIBMOTIONCAPTURE_ENABLE_NOKOV OFF)

# To use Motion Analysis, place their SDK in
# motion_capture_tracking/deps/libmotioncapture/deps/cortex_sdk_linux
# and set this variable to "ON"
set(LIBMOTIONCAPTURE_ENABLE_MOTIONANALYSIS OFF)

add_subdirectory(deps/libmotioncapture EXCLUDE_FROM_ALL)

include_directories(
  deps/libmotioncapture/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(motioncapture_node src/motioncapture_node.cpp)

target_link_libraries(motioncapture_node
  libmotioncapture
)
ament_target_dependencies(motioncapture_node
  rclcpp
  tf2_ros
  sensor_msgs
)

target_include_directories(motioncapture_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motioncapture_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS motioncapture_node
  DESTINATION lib/${PROJECT_NAME})

# The precompiled libNatNet library is only available for x64 Linux
if ((CMAKE_SYSTEM_NAME STREQUAL "Linux") AND (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64"))
  install(FILES 
    ${CMAKE_CURRENT_SOURCE_DIR}/deps/libmotioncapture/deps/NatNetSDKCrossplatform/lib/ubuntu/libNatNet.so
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# Python node
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/motioncapture_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files.
#install(DIRECTORY
#  config
#  DESTINATION share/${PROJECT_NAME}/
#)

ament_package()
