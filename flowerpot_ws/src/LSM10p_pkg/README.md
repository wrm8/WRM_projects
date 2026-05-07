# lslidar

## Description
The `lslidar package is a linux ROS2 driver for lslidar M10 ,M10_GPS,M10_P,M10_PLUS and N10.
The package is tested on Ubuntu 20.04 with ROS2 indigo.

## Compling
This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compling a catkin package will work.

```
cd your_work_space
colcon build
source install/setup.bash
ros2 launch lslidar_driver lslidar_launch.py
```
open new terminal
ros2 topic pub -1 /lslidar_order std_msgs/msg/Int8 data:\ 1\ 		(open radar)
ros2 topic pub -1 /lslidar_order std_msgs/msg/Int8 data:\ 0\ 		(close radar)


#编译
colcon build --packages-select lslidar_driver lslidar_msgs radar_msgs radar_pkg
#运行串口版
ros2 launch lslidar_driver lsm10p_uart_launch.py
ros2 launch lslidar_driver lslidar_launch.py

```

Note that this launch file launches both the driver, which is the only launch file needed to be used.


## FAQ


## Bug Report

Prefer to open an issue. You can also send an E-mail to honghangli@lslidar.com




RERTION 


##SSH/export DISPLAY=:0/ros2 run rviz2 rviz2

#运行可视化
python3 src/radar_pkg/scripts/lidar_visualize.py

# 获取一帧完整的激光雷达数据
ros2 topic echo /scan --once
# 保存完整的一帧数据（包括所有角度）
ros2 topic echo /scan --once --field ranges > scan_360.txt
#保存一帧所有的数据，不删减
ros2 topic echo /scan --once --full > scan_full.txt

# 查看有多少个点（应该是 360度 / 角度增量）
wc -l scan_360.txt

#动态调参
# 调整固定半径范围（根据实际花盆大小）
ros2 param set /radar_node fixed_min_radius 0.06
ros2 param set /radar_node fixed_max_radius 0.22

# 调整墙角过滤灵敏度（比值越小越容易过滤直线，0.1表示非常敏感）
ros2 param set /radar_node wall_eigenvalue_ratio 0.15

# 关闭墙角过滤
ros2 param set /radar_node enable_corner_filter false

# 调整检测距离范围
ros2 param set /radar_node max_range 5.0

# 调整自适应分段阈值
ros2 param set /radar_node near_thresh 0.6
ros2 param set /radar_node mid_thresh 1.5
