cmake_minimum_required(VERSION 3.8)
project(lgs_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp)
find_package(Threads REQUIRED)

add_executable(behavior_tree 
  src/main.cpp
  src/Ros.cpp
)

target_include_directories(behavior_tree PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(behavior_tree PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# ROS
ament_target_dependencies(behavior_tree
  rclcpp 
  std_msgs 
  sensor_msgs 
)

install(TARGETS behavior_tree
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  launch/launch.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME})

target_link_libraries(behavior_tree
  BT::behaviortree_cpp
  Threads::Threads
) 

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

ament_package()
