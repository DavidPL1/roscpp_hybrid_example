file(MAKE_DIRECTORY ${GENERATED_HEADERS_DIR})
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ros_version.h.in
  ${GENERATED_HEADERS_DIR}/ros_version.h
)
