cmake_minimum_required(VERSION 3.8)
project(zed_sdk_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_library(SL_ZED_LIBRARY NAMES sl_zed PATHS /usr/local/zed/lib REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)


add_executable(zed_sdk_cpp 
 src/zed_sdk.cpp
)

# Use the correct include paths
target_include_directories(zed_sdk_cpp PRIVATE
  /usr/local/zed/include
  /usr/local/cuda/include
)

target_link_libraries(zed_sdk_cpp ${SL_ZED_LIBRARY})

ament_target_dependencies(zed_sdk_cpp
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge
)

# Instalação
install(TARGETS
  zed_sdk_cpp
  DESTINATION lib/${PROJECT_NAME}
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
