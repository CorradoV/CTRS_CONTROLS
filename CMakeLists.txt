cmake_minimum_required(VERSION 3.8)
project(ctr_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include/ctr_teleop)

add_library(ctcr_libkine src/ctcr_model.cpp)

ament_export_targets(ctcr_libkine HAS_LIBRARY_TARGET)

add_executable(joy_wrap src/JOY_WRAP_ROS_2.cpp)
add_executable(ctr_sim src/CTR_SIM.cpp )
ament_target_dependencies(joy_wrap rclcpp std_msgs sensor_msgs geometry_msgs)
ament_target_dependencies(ctr_sim rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs)
target_link_libraries(ctr_sim ctcr_libkine tf2::tf2)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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



install(
  DIRECTORY include/ctr_teleop
  DESTINATION include
)

install(TARGETS
  joy_wrap
  ctr_sim
  ctcr_libkine
  DESTINATION lib/${PROJECT_NAME}
  EXPORT ctcr_libkine
  #LIBRARY DESTINATION lib
  #ARCHIVE DESTINATION lib
  #RUNTIME DESTINATION bin
  #INCLUDES DESTINATION include)
  )

ament_package()
