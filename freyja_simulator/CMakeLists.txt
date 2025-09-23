cmake_minimum_required(VERSION 3.20)
project(freyja_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-reorder)
endif()

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-O3)# -fno-math-errno -fno-trapping-math)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(freyja_msgs REQUIRED)
#find_package(mavros_msgs REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_library(zmqpp-s NAMES libzmqpp.so
  PATHS /opt/ros/kilted/lib/
  REQUIRED)

link_directories(/opt/ros/kilted/lib)

add_subdirectory(extern/flightlib)

add_executable(freyja_simulator_node src/freyja_simulator_node.cpp)
target_include_directories(freyja_simulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/extern/flightlib/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(freyja_simulator_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(freyja_simulator_node
  flightlib
  zmqpp
)
ament_target_dependencies(
  freyja_simulator_node
  "rclcpp"
  "Eigen3"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "nav_msgs"
  "std_msgs"
  "std_srvs"
  "sensor_msgs"
  "visualization_msgs"
  freyja_msgs
  # mavros_msgs
)

install(TARGETS freyja_simulator_node
  DESTINATION lib/${PROJECT_NAME})



ament_package()
