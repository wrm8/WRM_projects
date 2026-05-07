cmake_minimum_required(VERSION 3.8)
project(yolo11_pkg)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yolo11_msgs REQUIRED)

# 安装 Python 脚本
install(PROGRAMS
  scripts/onnx.py
  scripts/Coordinate_Transformation.py
  scripts/take_photo.py
  scripts/camera.sh
  DESTINATION lib/${PROJECT_NAME}
)

# 重要：安装模型文件和配置文件
install(FILES
  scripts/best.onnx
  scripts/flower.yaml
  DESTINATION share/${PROJECT_NAME}/scripts
)

# 安装 launch 文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()