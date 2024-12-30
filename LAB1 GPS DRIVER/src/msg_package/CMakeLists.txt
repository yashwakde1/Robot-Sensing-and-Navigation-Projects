cmake_minimum_required(VERSION 3.8)
project(msg_package)

# Add compile options for warnings in GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Specify the message files to be generated
set(msg_files
  "msg/GPSmsg.msg"
)

# Generate the interfaces (messages/services) from the .msg file
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} 
  DEPENDENCIES geometry_msgs std_msgs  # Ensure dependencies are declared correctly
)

# Optionally, add more dependencies here if needed
# find_package(<dependency> REQUIRED)

# Enable testing and linting if required
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright check (adjust as needed)
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint if not in a git repo (adjust if needed)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Declare this as ament package
ament_package()
