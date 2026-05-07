cmake_minimum_required(VERSION 3.8)
project(radar_msgs)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RadarTarget.msg"
  "msg/RadarTargetArray.msg"
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)

# 安装
ament_package()
