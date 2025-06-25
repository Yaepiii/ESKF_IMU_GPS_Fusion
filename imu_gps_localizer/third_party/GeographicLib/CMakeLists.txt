cmake_minimum_required(VERSION 3.5)
project(GeographicLib VERSION 1.49.0 LANGUAGES CXX)

# 添加PIC编译选项
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 设置默认精度
set(GEOGRAPHICLIB_PRECISION 2 CACHE STRING
  "Precision: 1 = float, 2 = double, 3 = extended, 4 = quadruple, 5 = variable")
set_property(CACHE GEOGRAPHICLIB_PRECISION PROPERTY STRINGS 1 2 3 4 5)

# 定义共享库而不是静态库
add_library(GeographicLib SHARED
  src/LocalCartesian.cpp
  src/Geocentric.cpp
  src/Math.cpp
)

# 设置别名
add_library(GeographicLib::GeographicLib ALIAS GeographicLib)

# 包含目录
target_include_directories(GeographicLib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 版本属性
set_target_properties(GeographicLib PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  PUBLIC_HEADER "include/Geocentric.hpp;include/LocalCartesian.hpp;include/Math.hpp"
)
