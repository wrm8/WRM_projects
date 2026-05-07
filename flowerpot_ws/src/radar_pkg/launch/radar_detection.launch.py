from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch():
    return LaunchDescription([
        # C++ 雷达处理节点
        Node(
            package="radar_pkg",
            executable="radar_node",
            name="radar_process_node",
            output="screen"
        ),

        # Python 可视化节点
        Node(
            package="radar_pkg",
            executable="lidar_visualize.py",
            name="lidar_visualize_node",
            output="screen"
        )
    ])