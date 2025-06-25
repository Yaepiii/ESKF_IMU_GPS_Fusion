cmake_minimum_required(VERSION 3.8)
project(fusion_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_subdirectory(imu_gps_localizer)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(fusion_localization src/fusion_localization.cpp)
target_include_directories(fusion_localization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  imu_gps_localizer/include  # 添加imu_gps_localizer的头文件路径
)
target_compile_features(fusion_localization PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  fusion_localization
  rclcpp
  nav_msgs
  geometry_msgs
  Eigen3
  tf2_ros
  visualization_msgs
  sensor_msgs
)
# 手动链接imu_gps_localizer_lib
target_link_libraries(fusion_localization
  imu_gps_localizer_lib
)

install(TARGETS fusion_localization
  DESTINATION lib/${PROJECT_NAME})

add_executable(time_align src/time_align.cpp)
target_include_directories(time_align PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
target_compile_features(time_align PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  time_align
  "pcl_ros"
  "pcl_conversions"
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "Eigen3"
  "tf2_ros"
)

install(TARGETS time_align
  DESTINATION lib/${PROJECT_NAME})

# 安装 launch 文件夹到 share 目录
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
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
