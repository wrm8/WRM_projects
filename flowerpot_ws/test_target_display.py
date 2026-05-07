#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from radar_msgs.msg import RadarTargetArray
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import math

class TargetTester(Node):
    def __init__(self):
        super().__init__('target_tester')
        self.sub = self.create_subscription(RadarTargetArray, '/radar_targets', self.callback, 10)
        self.targets = []
        
        # 创建图形
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(-3, 3)
        self.ax.set_aspect('equal')
        self.ax.set_title('Flower Pot Detection Test')
        self.ax.grid(True)
        self.ax.axhline(y=0, color='k', linewidth=0.5)
        self.ax.axvline(x=0, color='k', linewidth=0.5)
        
        self.circles = []
        self.get_logger().info('测试节点启动，等待花盆数据...')
        
        self.ani = animation.FuncAnimation(self.fig, self.update, interval=100, blit=False)
        plt.show()
    
    def callback(self, msg):
        self.targets = []
        self.get_logger().info(f'收到目标数据，共 {len(msg.array)} 个目标')
        
        for i, target in enumerate(msg.array):
            x = target.r * math.cos(target.phi)
            y = target.r * math.sin(target.phi)
            self.targets.append((x, y, target.r))
            self.get_logger().info(f'  目标{i+1}: 距离={target.r:.2f}m, 角度={target.phi*180/math.pi:.1f}°, 位置=({x:.2f}, {y:.2f})')
    
    def update(self, frame):
        # 清除旧圆圈
        for c in self.circles:
            c.remove()
        self.circles.clear()
        
        # 绘制新圆圈
        for x, y, r in self.targets:
            # 画大圆圈，确保可见
            circle = Circle((x, y), 0.25, color='red', fill=False, linewidth=3)
            self.ax.add_patch(circle)
            self.circles.append(circle)
            
            # 画圆心
            center = self.ax.plot(x, y, 'r+', markersize=15, markeredgewidth=3)
            self.circles.extend(center)
            
            # 添加文本标签
            text = self.ax.text(x, y+0.3, f'{r:.2f}m', fontsize=12, ha='center',
                               bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.8))
            self.circles.append(text)
            
            # 添加坐标标签
            coord = self.ax.text(x, y-0.3, f'({x:.2f},{y:.2f})', fontsize=10, ha='center',
                                bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
            self.circles.append(coord)
        
        return self.circles

def main(args=None):
    rclpy.init(args=args)
    node = TargetTester()
    
    import threading
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    print("=" * 50)
    print("花盆显示测试工具")
    print("请确保雷达节点正在运行")
    print("=" * 50)
    main()
