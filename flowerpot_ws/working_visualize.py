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

class WorkingVisualizer(Node):
    def __init__(self):
        super().__init__('working_visualizer')
        
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_target = self.create_subscription(RadarTargetArray, '/radar_targets', self.target_callback, 10)
        
        self.scan_points = np.zeros((0, 2))
        self.targets = []
        
        # 设置图形
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(-3, 3)
        self.ax.set_aspect('equal')
        self.ax.set_title('Working Visualization - Flower Pots')
        self.ax.grid(True)
        
        self.scat = self.ax.scatter([], [], c='blue', s=2, alpha=0.6, label='LiDAR')
        self.circles = []
        
        self.ax.legend()
        plt.ion()
        plt.show()
        
        self.get_logger().info('可视化启动')
    
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
            self.scat.set_offsets(self.scan_points)
            self.fig.canvas.draw_idle()
    
    def target_callback(self, msg):
        self.get_logger().info(f'收到 {len(msg.array)} 个目标')
        self.targets = []
        for target in msg.array:
            x = target.r * math.cos(target.phi)
            y = target.r * math.sin(target.phi)
            self.targets.append((x, y, target.r))
            self.get_logger().info(f'  花盆: ({x:.2f}, {y:.2f}) 距离={target.r:.2f}m')
        
        # 更新圆圈
        for c in self.circles:
            c.remove()
        self.circles.clear()
        
        for x, y, r in self.targets:
            circle = Circle((x, y), 0.2, color='red', fill=False, linewidth=2)
            self.ax.add_patch(circle)
            self.circles.append(circle)
            self.ax.plot(x, y, 'r+', markersize=10)
        
        self.fig.canvas.draw_idle()

def main():
    rclpy.init()
    node = WorkingVisualizer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
