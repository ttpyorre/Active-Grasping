cmake_minimum_required(VERSION 3.8)
project(vbm_project_env)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(pc src/pc.cpp)
ament_target_dependencies(pc rclcpp std_msgs geometry_msgs sensor_msgs)


install(DIRECTORY launch worlds urdf
  DESTINATION share/${PROJECT_NAME}) 
  
install(TARGETS
 
  pc
  DESTINATION lib/${PROJECT_NAME}) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
