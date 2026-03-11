cmake_minimum_required(VERSION 3.8)
project(frenet_lattice_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


find_package(rclcpp REQUIRED) # ROS2 C++ core library
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ackermann_msgs REQUIRED)

include_directories(include)  # Add include directory
add_executable(frenet_planner_node src/frenet_planner.cpp) # Create executable

ament_target_dependencies(frenet_planner_node 
                          rclcpp 
                          nav_msgs
                          visualization_msgs
                          geometry_msgs
                          sensor_msgs
                          tf2
                          ackermann_msgs
                          ) # Link ROS2 dependencies

target_link_libraries(frenet_planner_node Eigen3::Eigen)

install(TARGETS
    frenet_planner_node
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
