cmake_minimum_required(VERSION 3.8)
project(cpp_hybrid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
else()
  message(FATAL "ROS_VERSION environment variable is not set!")
endif()


list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(${ROS_VERSION} EQUAL "2")
  include(ROS2)
else()
  include(ROS1)
endif()

add_custom_command(
  OUTPUT ${GENERATED_HEADERS_DIR}/${PROJECT_NAME}/ros_version.h always_rebuild
  COMMAND ${CMAKE_COMMAND}
  -DROS_VERSION=${ROS_VERSION}
  -DGENERATED_HEADERS_DIR=${GENERATED_HEADERS_DIR}/${PROJECT_NAME}
  -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_ros_version_h.cmake
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(generate_ros_version_h
  DEPENDS always_rebuild
  COMMENT "Generating ros_version.h"
)

add_dependencies(talker generate_ros_version_h)
add_dependencies(listener generate_ros_version_h)
add_dependencies(server generate_ros_version_h)
add_dependencies(client generate_ros_version_h)
install(FILES
  ${GENERATED_HEADERS_DIR}/${PROJECT_NAME}/ros_version.h
  DESTINATION ${GENERATED_HEADERS_INSTALL_DIR}
)