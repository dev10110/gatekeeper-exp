cmake_minimum_required(VERSION 3.5)
project(gatekeeper)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


#add_compile_options(-O3 -march=native -DNDEBUG -UDEBUG)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dasc_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(octomap REQUIRED)
find_package(px4_msgs REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(nominal_controller src/nominal_controller.cpp)
ament_target_dependencies(nominal_controller
  dasc_msgs
  sensor_msgs
  builtin_interfaces
  tf2
  tf2_ros
  rclcpp
  nav_msgs
  geometry_msgs
)
install(TARGETS
  nominal_controller
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(joystick_controller src/joystick_controller.cpp)
ament_target_dependencies(joystick_controller
  dasc_msgs
  sensor_msgs
  builtin_interfaces
  tf2
  tf2_ros
  rclcpp
  visualization_msgs
  nav_msgs
  geometry_msgs
)
install(TARGETS
  joystick_controller
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(gatekeeper
       	src/gatekeeper.cpp
	src/transforms.cpp
  src/dynamics.cpp
)
ament_target_dependencies(gatekeeper
  dasc_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  rclcpp
  dasc_msgs
  nav_msgs
  px4_msgs
)


target_link_libraries(gatekeeper
  ${PCL_LIBRARIES} 
  ${OCTOMAP_LIBRARIES}
)


install(TARGETS
  gatekeeper
  DESTINATION lib/${PROJECT_NAME}
)




ament_package()
