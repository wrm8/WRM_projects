#!/usr/bin/env python3
import matplotlib
matplotlib.use('TkAgg')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from radar_msgs.msg import RadarTargetArray
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import math

# ========== 可调显示参数（与C++节点保持一致）==========
# 角度范围（弧度）  默认前方120度：-60° 到 +60°
ANGLE_MIN =  20 * math.pi/180   # -60°
ANGLE_MAX =  140 * math.pi/180   # +60°
# 坐标轴范围（米）
X_LIM = (-2.0, 2.0)      # X轴显示范围
Y_LIM = (-2.0, 2.0)      # Y轴显示范围
# 是否自动调整坐标轴范围（根据实际点云）
AUTO_AXIS = False        # 若为True，则动态调整xlim/ylim

class LidarVisualizeNode(Node):
    def __init__(self):
        super().__init__("lidar_visualize_node")
        
        # 订阅话题
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.sub_target = self.create_subscription(RadarTargetArray, "/radar_targets", self.target_callback, 10)
        
        self.scan_points = np.empty((0, 2))
        self.pots = []  # (x, y, radius, id)
        
        # 创建图形
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_xlim(X_LIM)
        self.ax.set_ylim(Y_LIM)
        self.ax.set_aspect('equal')
        self.ax.set_title(f'Flower Pot Detection (Angle: {ANGLE_MIN*180/math.pi:.0f}°~{ANGLE_MAX*180/math.pi:.0f}°)')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_facecolor('white')
        
        # 点云散点图
        self.scatter = self.ax.scatter([], [], c='blue', s=5, alpha=0.6, label='LiDAR Points')
        self.ax.axhline(y=0, color='k', linewidth=0.5, alpha=0.5)
        self.ax.axvline(x=0, color='k', linewidth=0.5, alpha=0.5)
        self.ax.legend(loc='upper right')
        
        self.dynamic_artists = []
        plt.show()
        self.get_logger().info(f"可视化节点启动 (角度范围: {ANGLE_MIN*180/math.pi:.0f}° ~ {ANGLE_MAX*180/math.pi:.0f}°)")
    
    def scan_callback(self, msg):
        # 获取原始点云
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        
        # 距离过滤
        valid = (ranges > 0.15) & (ranges < 3.0) & np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]
        
        # 角度过滤（使用可调范围）
        angle_mask = (angles >= ANGLE_MIN) & (angles <= ANGLE_MAX)
        ranges = ranges[angle_mask]
        angles = angles[angle_mask]
        
        if len(ranges) > 0:
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)
            self.scan_points = np.column_stack((x, y))
            self.scatter.set_offsets(self.scan_points)
            
            # 可选：自动调整坐标轴范围
            if AUTO_AXIS and len(self.scan_points) > 0:
                x_min, x_max = np.min(x), np.max(x)
                y_min, y_max = np.min(y), np.max(y)
                margin = 0.2
                self.ax.set_xlim(x_min - margin, x_max + margin)
                self.ax.set_ylim(y_min - margin, y_max + margin)
                self.ax.set_aspect('equal')
        
        self.redraw()
    
    def target_callback(self, msg):
        self.pots = [(t.x, t.y, t.radius, t.n) for t in msg.array]
        self.redraw()
    
    def redraw(self):
        for a in self.dynamic_artists:
            a.remove()
        self.dynamic_artists.clear()
        
        for x, y, radius, pid in self.pots:
            circle = Circle((x, y), radius, color='red', fill=False, linewidth=2)
            self.ax.add_patch(circle)
            self.dynamic_artists.append(circle)
            
            center = self.ax.plot(x, y, 'g+', markersize=10, markeredgewidth=2)
            self.dynamic_artists.extend(center)
            
            label = self.ax.text(x, y+0.15, f'ID:{pid} {radius:.2f}m', 
                                 fontsize=8, ha='center',
                                 bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
            self.dynamic_artists.append(label)
        
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = LidarVisualizeNode()
    rclpy.spin(node)

if __name__ == "__main__":
    print("=" * 50)
    print("雷达可视化节点启动 (支持可调角度/坐标轴范围)")
    print("确保雷达节点正在运行: ros2 run radar_pkg radar_node")
    print(f"当前角度范围: {ANGLE_MIN*180/math.pi:.0f}° ~ {ANGLE_MAX*180/math.pi:.0f}°")
    print("如需修改，请编辑脚本开头的 ANGLE_MIN, ANGLE_MAX 变量")
    print("=" * 50)
    main()