from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 自动获取包路径
    yolo_pkg_path = get_package_share_directory('yolo11_pkg')
    model_path = os.path.join(yolo_pkg_path, '../scripts/best.onnx')
    yaml_path = os.path.join(yolo_pkg_path, '../scripts/flower.yaml')

    return LaunchDescription([
        Node(
            package='yolo11_pkg',
            executable='onnx.py',
            name='yolo11_node',
            output='screen',
            respawn=True,
            # 传入模型 + 配置 + 显示画面
            arguments=[
                '--model', model_path,
                '--yaml', yaml_path,
                '--show'
            ],
            parameters=[{'show': True}]
        ),
    ])