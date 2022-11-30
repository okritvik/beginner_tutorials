cmake_minimum_required(VERSION 3.8)
project(ros2_cpp_pubsub)

# clangd support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
execute_process(COMMAND "rm" "-f" "${CMAKE_SOURCE_DIR}/compile_commands.json")
execute_process(COMMAND "ln" "-s" "-r" "${CMAKE_BINARY_DIR}/compile_commands.json" "${CMAKE_SOURCE_DIR}/")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# For service and msg
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Data.msg"
  "srv/ChangeString.srv"
  DEPENDENCIES std_msgs

)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
rosidl_target_interfaces(talker ${PROJECT_NAME} "rosidl_typesupport_cpp")
# TO make sure that executable files can link to service and message headers
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(talker "${cpp_typesupport_target}")


add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
# TO make sure that executable files can link to service and message headers
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(listener "${cpp_typesupport_target}")

add_executable(server src/change_string_server.cpp)
ament_target_dependencies(server rclcpp std_msgs)
# TO make sure that executable files can link to service and message headers
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(server "${cpp_typesupport_target}")

add_executable(client_pub src/client_publisher.cpp)
ament_target_dependencies(client_pub rclcpp std_msgs)
# TO make sure that executable files can link to service and message headers
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(client_pub "${cpp_typesupport_target}")

add_executable(client_sub src/subscriber_client_pub.cpp)
ament_target_dependencies(client_sub rclcpp std_msgs)
# TO make sure that executable files can link to service and message headers
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(client_sub "${cpp_typesupport_target}")

add_executable(param_talker src/param_pub.cpp)
ament_target_dependencies(param_talker rclcpp std_msgs)
# TO make sure that executable files can link to service and message headers
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(param_talker "${cpp_typesupport_target}")

add_executable(param_listener src/param_sub.cpp)
ament_target_dependencies(param_listener rclcpp std_msgs)
# TO make sure that executable files can link to service and message headers
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(param_listener "${cpp_typesupport_target}")

add_executable(static_transform_publisher src/publisher_static_transform.cpp)
ament_target_dependencies(static_transform_publisher geometry_msgs rclcpp std_msgs tf2 tf2_ros)
# TO make sure that executable files can link to service and message headers
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(static_transform_publisher "${cpp_typesupport_target}")

install(TARGETS
  talker
  listener
  server
  client_pub
  client_sub
  param_talker
  param_listener
  static_transform_publisher
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
