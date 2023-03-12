find_package(control_msgs REQUIRED)
find_package(drake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

set(HEADERS
  "follow_joint_trajectory_server.h"
)

# Mock install headers so include paths match installed paths
set(mock_include_dir "${CMAKE_CURRENT_BINARY_DIR}/include")
file(MAKE_DIRECTORY "${mock_include_dir}/drake_ros/action")
foreach(hdr ${HEADERS})
  configure_file("${hdr}" "${mock_include_dir}/drake_ros/action/${hdr}" COPYONLY)
endforeach()

add_library(drake_ros_action
  follow_joint_trajectory_server.cc
)

target_link_libraries(drake_ros_action PUBLIC
  ${control_msgs_TARGETS}
  drake_ros_core
  drake::drake
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

target_include_directories(drake_ros_action
  PUBLIC
    "$<BUILD_INTERFACE:${mock_include_dir}>"
    "$<INSTALL_INTERFACE:include>"
)

install(
  TARGETS drake_ros_action
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES
    ${HEADERS}
  DESTINATION include/drake_ros/action
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_follow_joint_trajectory_server 
    test/test_follow_joint_trajectory_server.cc)
  target_include_directories(test_follow_joint_trajectory_server
    PRIVATE
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  )
  target_link_libraries(test_follow_joint_trajectory_server
    drake::drake
    drake_ros_action
  )
endif()
