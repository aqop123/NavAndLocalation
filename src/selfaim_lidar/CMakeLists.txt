cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0074 NEW)
project(selfaim_lidar)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(livox_ros_driver2 REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(pcl_ros REQUIRED)
# find_package(PCL REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(pcl_conversions REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# target_include_directories(customMsg2scan PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(customMsg2scan PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# ament_target_dependencies(
#   customMsg2scan
#   "rclcpp"
#   "livox_ros_driver2"
#   "std_msgs"
#   "pcl_ros"
#   "PCL"
#   "nav_msgs"
#   "sensor_msgs"
#   "geometry_msgs"
#   "tf2"
#   "tf2_geometry_msgs"
#   "Eigen3"
#   "pcl_conversions"
# )

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/customMsg2scan_ver2.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "auto_aim::CustomPoint2PointCloud"
  EXECUTABLE ${PROJECT_NAME}_node
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

ament_auto_package(
  INSTALL_TO_SHARE

  launch
)
