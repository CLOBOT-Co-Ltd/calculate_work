cmake_minimum_required(VERSION 3.8)
project(pcd_filter)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# CMake 정책 설정
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED) 
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 실행 파일 추가
add_executable(pointcloud_filter_node src/pcd_filter.cpp)

# 인클루드 디렉토리 설정
target_include_directories(pointcloud_filter_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

# 타겟 링크 라이브러리 설정
target_link_libraries(pointcloud_filter_node
  ${PCL_LIBRARIES}
)

# ament 패키지 의존성 설정
ament_target_dependencies(pointcloud_filter_node
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_sensor_msgs
)

# 설치 설정
install(TARGETS pointcloud_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
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
