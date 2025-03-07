cmake_minimum_required(VERSION 3.5)
project(trajectory_visualization)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveTrajectory.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

include_directories(include)

# Publisher and Saver Node
add_executable(trajectory_publisher_saver src/trajectory_publisher_saver.cpp)
ament_target_dependencies(trajectory_publisher_saver rclcpp std_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros)

# Reader and Publisher Node
add_executable(trajectory_reader_publisher src/trajectory_reader_publisher.cpp)
ament_target_dependencies(trajectory_reader_publisher rclcpp std_msgs visualization_msgs geometry_msgs tf2 tf2_ros)

# Link generated interfaces.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(trajectory_publisher_saver ${cpp_typesupport_target})
target_link_libraries(trajectory_reader_publisher ${cpp_typesupport_target})

install(TARGETS
  trajectory_publisher_saver
  trajectory_reader_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
