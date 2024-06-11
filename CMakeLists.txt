cmake_minimum_required(VERSION 3.8)
project(as2_platform_ardupilot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#----------------------------------------------------------------------------
# dependencies

find_package(ament_cmake REQUIRED)
find_package(as2_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(as2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

if (APPLE)
  # macOS Sonoma 14.5 aarch64 transitive dependency issue
  find_package(GeographicLib REQUIRED)
  find_package(yaml-cpp REQUIRED)

  # assuming yaml-cpp installed with brew
  if (CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
    link_directories("/opt/homebrew/opt/yaml-cpp/lib")
  elseif (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    link_directories("/usr/local/opt/yaml-cpp/lib")
  endif()

endif()

#----------------------------------------------------------------------------
# build

add_executable(${PROJECT_NAME}_node
  src/ardupilot_platform_main.cpp
  src/ardupilot_platform.cpp
)

target_include_directories(${PROJECT_NAME}_node
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"  
)

target_link_libraries(${PROJECT_NAME}_node
  as2_core::as2_core
  GeographicLib::GeographicLib
)

#----------------------------------------------------------------------------
# testing

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

#----------------------------------------------------------------------------
# install

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

#----------------------------------------------------------------------------
# ament_package last

ament_package()
