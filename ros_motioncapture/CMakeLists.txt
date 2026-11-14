cmake_minimum_required(VERSION 3.10)
project(ros_motioncapture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INSTALL_RPATH "$ORIGIN")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Threads REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# add dependencies
set(LIBMOTIONCAPTURE_BUILD_PYTHON_BINDINGS OFF)
set(LIBMOTIONCAPTURE_BUILD_EXAMPLE OFF)

option(LIBMOTIONCAPTURE_ENABLE_QUALISYS "Build QUALISYS backend" OFF)
option(LIBMOTIONCAPTURE_ENABLE_OPTITRACK "Build OPTITRACK backend" ON)
option(LIBMOTIONCAPTURE_ENABLE_OPTITRACK_CLOSED_SOURCE "Build OPTITRACK_CLOSED_SOURCE backend" OFF)
option(LIBMOTIONCAPTURE_ENABLE_VICON "Build VICON backend" ON)
option(LIBMOTIONCAPTURE_ENABLE_VRPN "Build VRPN backend" OFF)
set(VRPN_BUILD_SERVERS OFF CACHE BOOL "Build standalone VRPN tools")
set(VRPN_BUILD_SERVER_LIBRARY ${BUILD_TESTING})
option(LIBMOTIONCAPTURE_ENABLE_FZMOTION "Build FZMOTION backend" OFF)

# To use NOKOV, place their SDK in
# ros_motioncapture/deps/libmotioncapture/deps/nokov_sdk
# and set this variable to "ON"
option(LIBMOTIONCAPTURE_ENABLE_NOKOV "Build NOKOV backend" OFF)

# To use Motion Analysis, place their SDK in
# ros_motioncapture/deps/libmotioncapture/deps/cortex_sdk_linux
# and set this variable to "ON"
option(LIBMOTIONCAPTURE_ENABLE_MOTIONANALYSIS "Build MOTIONANALYSIS backend" OFF)

# VRPN's bundled cppcheck tests target an older cppcheck command line.
set(CMAKE_DISABLE_FIND_PACKAGE_cppcheck TRUE)
add_subdirectory(deps/libmotioncapture EXCLUDE_FROM_ALL)
unset(CMAKE_DISABLE_FIND_PACKAGE_cppcheck)

include_directories(
  deps/libmotioncapture/include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(motioncapture_node src/motioncapture_node.cpp src/motioncapture_diagnostics.cpp)

target_link_libraries(motioncapture_node
  libmotioncapture
  Threads::Threads
)
target_link_libraries(motioncapture_node
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  diagnostic_msgs::diagnostic_msgs
  sensor_msgs::sensor_msgs
)

target_include_directories(motioncapture_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motioncapture_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS libmotioncapture
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(LIBMOTIONCAPTURE_ENABLE_VRPN)
  install(TARGETS vrpn quat
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}/vrpn EXCLUDE_FROM_ALL)
endif()

install(TARGETS motioncapture_node
  DESTINATION lib/${PROJECT_NAME})

# The precompiled libNatNet library is only available for x64 Linux
if ((CMAKE_SYSTEM_NAME STREQUAL "Linux") AND (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64"))
  install(FILES 
    ${CMAKE_CURRENT_SOURCE_DIR}/deps/libmotioncapture/deps/NatNetSDKCrossplatform/lib/ubuntu/libNatNet.so
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_mock.py TIMEOUT 30)
  if(LIBMOTIONCAPTURE_ENABLE_VRPN)
    add_executable(vrpn_test_server test/vrpn_test_server.cpp)
    target_include_directories(vrpn_test_server PRIVATE deps/libmotioncapture/deps/vrpn)
    target_link_libraries(vrpn_test_server vrpnserver Threads::Threads)
    add_launch_test(test/test_vrpn.py TIMEOUT 30
      ARGS "server:=$<TARGET_FILE:vrpn_test_server>")
  endif()
endif()

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
