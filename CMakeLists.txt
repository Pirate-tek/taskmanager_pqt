cmake_minimum_required(VERSION 3.8)
project(taskmanager_pqt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate Actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MissionTask.action"
  DEPENDENCIES geometry_msgs action_msgs
)

# Install Launch files
install(FILES
  launch/mission_launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install the control node
install(PROGRAMS
  taskmanager_pqt/control_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME control_node
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
