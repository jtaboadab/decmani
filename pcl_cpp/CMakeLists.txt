cmake_minimum_required(VERSION 3.8)
project(pcl_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(msg_srv_creator REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl src/pcl_cpp.cpp)
target_link_libraries (pcl ${PCL_LIBRARIES})
ament_target_dependencies(pcl rclcpp pcl_conversions pcl_ros std_msgs sensor_msgs msg_srv_creator OpenCV cv_bridge)

add_executable(pruebas src/pruebas_cpp.cpp)
target_link_libraries (pruebas ${PCL_LIBRARIES})
ament_target_dependencies(pruebas rclcpp pcl_conversions pcl_ros std_msgs sensor_msgs msg_srv_creator OpenCV cv_bridge)

install(TARGETS
  pcl
  pruebas
  DESTINATION lib/${PROJECT_NAME})

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
