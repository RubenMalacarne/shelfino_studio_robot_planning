cmake_minimum_required(VERSION 3.8)
project(planning_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(obstacles_msgs REQUIRED)
find_package(Boost 1.74)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

install(DIRECTORY 
        launch include
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
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

add_library(linear_path
  src/algo_path_planning/linear_path.cpp
)

add_executable(main_base
  src/main_base.cpp
)

add_executable(re_mapping
  src/utils/re_mapping.cpp
)

add_executable(nav2_pathclient
  src/utils/nav2_pathclient.cpp
)

add_executable(dubins_service
  src/utils/dubins_service.cpp
)

add_executable(path_orchestrator
  src/path_orchestrator.cpp
)

ament_target_dependencies(linear_path
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(main_base
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  obstacles_msgs
)

ament_target_dependencies(re_mapping
  rclcpp
  geometry_msgs
  obstacles_msgs
  tf2
  visualization_msgs
  std_srvs
)

ament_target_dependencies(dubins_service 
  rclcpp 
  std_srvs
  geometry_msgs
)

ament_target_dependencies(path_orchestrator 
  rclcpp 
  std_srvs
  geometry_msgs
  visualization_msgs
  obstacles_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(path_orchestrator linear_path)


ament_target_dependencies(nav2_pathclient
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_ros
  visualization_msgs
  obstacles_msgs
)
target_include_directories(main_base PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(re_mapping PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(nav2_pathclient PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(dubins_service PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(path_orchestrator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(linear_path PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)



install(TARGETS
main_base
re_mapping
nav2_pathclient
dubins_service
path_orchestrator
linear_path
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
