cmake_minimum_required(VERSION 3.8)
project(tutorial_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set ROS version option based on ROS_VERSION env variable
if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
else()
  message(FATAL "ROS_VERSION environment variable is not set!")
endif()

if (${ROS_VERSION} EQUAL "2") # ROS 2
  find_package(ament_cmake REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Num.msg"
    "msg/Sphere.msg"
    "srv/AddThreeInts.srv"
    "srv/AddTwoInts.srv"
    DEPENDENCIES geometry_msgs
  )

  ament_package()
else() # ROS 1

  find_package(catkin REQUIRED COMPONENTS
    message_generation
    geometry_msgs
  )

  add_message_files(
    FILES
    Num.msg
    Sphere.msg
  )

  add_service_files(
    FILES
    AddThreeInts.srv
    AddTwoInts.srv
  )

  generate_messages(
    DEPENDENCIES
    geometry_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
      message_runtime
      geometry_msgs
  )
endif()
