## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tutorial_interfaces)

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS
     roscpp
     std_msgs
     tutorial_interfaces
    INCLUDE_DIRS
     include
)

if(NOT DEFINED CATKIN_DEVEL_PREFIX)
  set(CATKIN_DEVEL_PREFIX ${CMAKE_CURRENT_BINARY_DIR})
endif()

include_directories(${catkin_INCLUDE_DIRS} ${CATKIN_DEVEL_PREFIX}/include)

add_executable(talker src/publisher.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/subscriber.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(server src/server.cpp)
target_link_libraries(server ${catkin_LIBRARIES})

add_executable(client src/client.cpp)
target_link_libraries(client ${catkin_LIBRARIES})

set(GENERATED_HEADERS_DIR "${CATKIN_DEVEL_PREFIX}/include")
set(GENERATED_HEADERS_INSTALL_DIR "${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}")

install(TARGETS talker listener server client
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
 )
