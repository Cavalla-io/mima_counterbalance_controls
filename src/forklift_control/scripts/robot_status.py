#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Float32, Bool, Int8
import json
import time


class RobotStatusAggregator(Node):
    def __init__(self):
        super().__init__('robot_status_aggregator')
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Status with defaults
        self.status = {
            'pallet_status': 0,
            'temperature': 0.0,
            'mode': 'unknown',
        }
        
        # Subscriptions
        self.create_subscription(Int8, '/pallet_status', self.pallete_stauts, qos)
        self.create_subscription(Float32, '/temperature', self.temp_cb, qos)
        self.create_subscription(Bool, '/forklift/drive_status', self.mode_cb, qos)
        
        # Publisher
        self.pub = self.create_publisher(String, '/robot_status', qos)
        self.timer = self.create_timer(0.1, self.publish)  # 10 Hz

    def pallete_stauts(self, msg):
        self.status['pallet_status'] = msg.data

    def temp_cb(self, msg):
        self.status['temperature'] = msg.data

    def mode_cb(self, msg):
        self.status['mode'] = msg.data

    def publish(self):
        msg = String()
        msg.data = json.dumps(self.status)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RobotStatusAggregator())
    rclpy.shutdown()


if __name__ == '__main__':
    main()