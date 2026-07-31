cmake_minimum_required(VERSION 3.10)
project(ros2_motion_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS and C++ dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MotionPlanningService.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
)

set(dependencies "rclcpp" "pluginlib" "builtin_interfaces" "std_msgs" "geometry_msgs" "nav_msgs" "visualization_msgs")

set(libraries "${Boost_PROGRAM_OPTIONS_LIBRARY}" "${ament_LIBRARIES}" "${cpp_typesupport_target}")

add_library(node_lib
  src/${PROJECT_NAME}/astar_node.cc
  src/${PROJECT_NAME}/rrt_node.cc
)
ament_target_dependencies(node_lib ${dependencies})
target_link_libraries(node_lib ${libraries})

add_library(util_lib
  src/${PROJECT_NAME}/utility_functions.cc
)
ament_target_dependencies(util_lib ${dependencies})
target_link_libraries(util_lib ${libraries})

add_library(planner_functions_lib
  src/${PROJECT_NAME}/planner_functions.cc
)
ament_target_dependencies(planner_functions_lib ${dependencies})
target_link_libraries(planner_functions_lib ${libraries} util_lib)

add_library(planner_lib
  src/${PROJECT_NAME}/astar_planner.cc
  src/${PROJECT_NAME}/rrt_planner.cc
)
ament_target_dependencies(planner_lib ${dependencies})
target_link_libraries(planner_lib ${libraries} node_lib util_lib planner_functions_lib)

set(custom_libraries "node_lib" "util_lib" "planner_functions_lib" "planner_lib")

add_executable(motion_planning_server src/motion_planning_server.cc)
ament_target_dependencies(motion_planning_server ${dependencies})
target_link_libraries(motion_planning_server ${libraries} ${custom_libraries})

add_executable(motion_planning_client src/motion_planning_client.cc)
ament_target_dependencies(motion_planning_client ${dependencies})
target_link_libraries(motion_planning_client ${libraries} ${custom_libraries})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  motion_planning_server
  motion_planning_client
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS node_lib util_lib planner_functions_lib planner_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies} Boost rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
