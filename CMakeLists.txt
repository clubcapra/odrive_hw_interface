cmake_minimum_required(VERSION 3.8)
project(odrive_hw_interface)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
# Add any other dependencies you might have

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${hardware_interface_INCLUDE_DIRS}
)

# Declare a library or executable
# Replace 'your_source_file.cpp' with the name of your source file
add_library(${PROJECT_NAME} SHARED
  src/your_source_file.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  # Add any other dependencies you linked above
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Export information to downstream packages
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp hardware_interface)
# Add any other exported dependencies

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
