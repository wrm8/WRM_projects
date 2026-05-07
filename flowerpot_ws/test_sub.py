#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from radar_msgs.msg import RadarTargetArray

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        
        self.sub_scan = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10
        )
        
        self.sub_target = self.create_subscription(
            RadarTargetArray,
            '/radar_targets',
            self.target_callback,
            10
        )
        
        self.get_logger().info('测试节点启动，等待数据...')
        self.scan_count = 0
        self.target_count = 0
    
    def scan_callback(self, msg):
        self.scan_count += 1
        self.get_logger().info(f'📡 收到Scan数据 #{self.scan_count}: {len(msg.ranges)} 个点')
    
    def target_callback(self, msg):
        self.target_count += 1
        self.get_logger().info(f'🎯 收到Target数据 #{self.target_count}: {len(msg.array)} 个目标')
        for target in msg.array:
            self.get_logger().info(f'   目标: r={target.r:.2f}m, phi={target.phi:.2f}rad ({target.phi*180/3.14159:.1f}度)')

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
