#camera
#ssh远程加载图形界面
export DISPLAY=:0
#把压缩图像转化为原始图像
ros2 run image_transport republish compressed raw --ros-args -r in/compressed:=/image_raw/compressed -r out:=/image_raw
#开启摄像头,不需要解压缩，直接出来个窗口，显示画面
ros2 run rqt_image_view rqt_image_view
#修改文件内容
cat > 文件精确地址 << 'EOF'
修改的内容
EOF

cat > ~/wrm/flower_robot-master_WRM/flowerpot_ws/src/radar_pkg/scripts/lidar_visualize.py << 'EOF'
修改的内容
EOF

cat > ~/wrm/flower_robot-master_WRM/flowerpot_ws/src/radar_pkg/src/radar_node.cpp << 'EOF'
修改的内容
EOF