cmake_minimum_required(VERSION 3.8)
project(gazebo_map_creator_harmonic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)

find_package(gz-transport13)
find_package(gz-msgs10)

add_library(MapParser SHARED src/map_parser.cpp)
set_property(TARGET MapParser PROPERTY CXX_STANDARD 17)
target_link_libraries(MapParser
  PRIVATE gz-plugin2::gz-plugin2
  PRIVATE gz-sim8::gz-sim8
  PRIVATE gz-transport13::core
  PRIVATE gz-msgs10::core
  )

add_executable(map_node src/map_node.cpp)
ament_target_dependencies(map_node
  rclcpp
  nav_msgs
  std_msgs
)
install(TARGETS MapParser
  LIBRARY DESTINATION lib)

install(TARGETS map_node
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
