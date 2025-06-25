cmake_minimum_required(VERSION 3.5)
project(imu_gps_localizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

# 添加GeographicLib
add_subdirectory(third_party/GeographicLib)

# 将库设为共享库
add_library(imu_gps_localizer_lib SHARED
    src/imu_gps_localizer.cpp
    src/initializer.cpp
    src/imu_processor.cpp
    src/gps_processor.cpp
)

target_include_directories(imu_gps_localizer_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    third_party/GeographicLib/include
)

target_link_libraries(imu_gps_localizer_lib
    ${EIGEN3_LIBS}
    GeographicLib::GeographicLib
)

install(TARGETS imu_gps_localizer_lib
    EXPORT imu_gps_localizer_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)