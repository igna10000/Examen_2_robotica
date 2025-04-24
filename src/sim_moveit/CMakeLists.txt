cmake_minimum_required(VERSION 3.8)
project(sim_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(sim_moveit src/sim_moveit.cpp)
add_executable(inv src/inv.cpp)
add_executable(direc src/direc.cpp)



target_include_directories(sim_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
  
 target_include_directories(inv PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_include_directories(direc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
  
target_compile_features(sim_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  sim_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
)



target_compile_features(inv PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  inv
  "moveit_ros_planning_interface"
  "rclcpp"
)

target_compile_features(direc PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  direc
  "moveit_ros_planning_interface"
  "rclcpp"
)


install(TARGETS sim_moveit
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS inv
DESTINATION lib/${PROJECT_NAME})

install(TARGETS direc
DESTINATION lib/${PROJECT_NAME})




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
