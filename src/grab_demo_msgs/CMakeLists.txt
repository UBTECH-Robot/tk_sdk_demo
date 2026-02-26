cmake_minimum_required(VERSION 3.8)
project(grab_demo_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# 声明消息文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IKRequest.msg"
  "msg/GraspCandidate.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
