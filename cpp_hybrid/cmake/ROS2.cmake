find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tutorial_interfaces REQUIRED)

include_directories(${CMAKE_BINARY_DIR}/include)

add_executable(talker src/publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(server src/server.cpp)
ament_target_dependencies(server rclcpp tutorial_interfaces)

add_executable(client src/client.cpp)
ament_target_dependencies(client rclcpp tutorial_interfaces)

install(TARGETS
  client
  server
  talker
  listener
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

set(GENERATED_HEADERS_DIR "${CMAKE_BINARY_DIR}/include")
set(GENERATED_HEADERS_INSTALL_DIR "${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}")