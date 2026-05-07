#!/usr/bin/env python3
import matplotlib
matplotlib.use('Qt5Agg')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from radar_msgs.msg import RadarTargetArray
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import math

class SimpleWorking(Node):
    def __init__(self):
        super().__init__('simple_working')
        
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_target = self.create_subscription(RadarTargetArray, '/radar_targets', self.target_callback, 10)
        
        self.scan_points = []
        self.targets = []
        
        # 创建图形 - 使用更简单的方式
        plt.ion()  # 交互模式
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111)
        
        # 设置坐标轴
        self.ax.set_xlim(-2, 3)
        self.ax.set_ylim(-2, 2)
        self.ax.set_aspect('equal')
        self.ax.set_title('Flower Pot Detection')
        self.ax.grid(True)
        self.ax.set_facecolor('white')  # 白色背景
        
        # 初始化绘图对象
        self.scatter = self.ax.scatter([], [], c='blue', s=10, alpha=0.6)
        
        self.fig.canvas.draw()
        plt.show(block=False)
        
        self.get_logger().info('可视化启动，等待数据...')
    
    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid = (ranges > 0.15) & (ranges < 3.0) & np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]
        front_mask = np.abs(angles) < math.pi/3
        ranges = ranges[front_mask]
        angles = angles[front_mask]
        
        if len(ranges) > 0:
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)
            self.scan_points = np.column_stack((x, y))
            
            # 更新显示
            self.scatter.set_offsets(self.scan_points)
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
    
    def target_callback(self, msg):
        # 清除旧圆圈
        for patch in self.ax.patches:
            if isinstance(patch, Circle):
                patch.remove()
        
        # 绘制新花盆
        for target in msg.array:
            x = target.r * math.cos(target.phi)
            y = target.r * math.sin(target.phi)
            
            self.get_logger().info(f'花盆: x={x:.2f}, y={y:.2f}, r={target.r:.2f}m')
            
            # 画圆圈
            circle = Circle((x, y), 0.15, color='red', fill=False, linewidth=2)
            self.ax.add_patch(circle)
            
            # 画中心点
            self.ax.plot(x, y, 'r+', markersize=10)
            
            # 添加文字
            self.ax.text(x, y+0.2, f'{target.r:.2f}m', fontsize=10, ha='center',
                        bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))
        
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

def main():
    rclpy.init()
    node = SimpleWorking()
    rclpy.spin(node)

if __name__ == '__main__':
    print("启动可视化...")
    main()
