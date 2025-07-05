cmake_minimum_required(VERSION 3.8)
project(sipeed_tof_ms_a010)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cJSON REQUIRED)

# --- Library: sipeed_tof_driver ---
add_library(sipeed_tof_driver
  src/serial_port.cpp
  src/frame_parser.cpp
)

target_include_directories(sipeed_tof_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- Executable: sipeed_tof_node ---
add_executable(sipeed_tof_node src/tof_node.cpp)

ament_target_dependencies(sipeed_tof_node PUBLIC
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
)

# Explicitly add the include directory for cJSON to the target
target_include_directories(sipeed_tof_node PRIVATE
  ${CJSON_INCLUDE_DIRS}
)

# Explicitly link all libraries to the final executable
target_link_libraries(sipeed_tof_node PRIVATE
  sipeed_tof_driver
  ${OpenCV_LIBRARIES}
  ${CJSON_LIBRARIES}
)

# --- Installation ---
install(TARGETS
  sipeed_tof_driver
  sipeed_tof_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# --- Testing ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()