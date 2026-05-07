#!/bin/bash
# 切换到conda环境
source ~/miniconda3/bin/activate yolo11  # 替换为你的conda路径和环境名

# 加载ROS环境
source /opt/ros/noetic/setup.bash  # 根据ROS版本调整
source ~/catkin_ws/devel/setup.bash # 根据工作空间调整

# 启动节点
rosrun yolo11_pkg onnx.py