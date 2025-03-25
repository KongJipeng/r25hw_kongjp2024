cmake_minimum_required(VERSION 3.8)
project(task3_icp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.8 REQUIRED)

# Include PCL directories
include_directories(
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 添加可执行文件
add_executable(icp_scan_matcher src/icp_scan_matcher.cpp)
ament_target_dependencies(icp_scan_matcher 
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  nav_msgs 
  tf2 
  tf2_ros 
  tf2_eigen 
  pcl_conversions
)
target_link_libraries(icp_scan_matcher ${PCL_LIBRARIES})

# 安装可执行文件
install(TARGETS
  icp_scan_matcher
  DESTINATION lib/${PROJECT_NAME})

# 安装launch文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# 创建输出目录
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/output
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
