cmake_minimum_required(VERSION 3.5)
project(arm_interfaces)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息和服务
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetJointAngle.srv"
  "srv/GetJointAngles.srv"
  DEPENDENCIES std_msgs
)

ament_package()
