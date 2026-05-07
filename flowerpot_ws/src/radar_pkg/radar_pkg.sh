# 自动进入工作空间（不管你在哪都能切对）
cd ~/wrm/flower_robot-master_WRM/flowerpot_ws
#编译
colcon build --packages-select lslidar_driver lslidar_msgs radar_msgs radar_pkg
#source
source install/setup.bash
#运行串口版
ros2 launch lslidar_driver lsm10p_uart_launch.py