#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import signal

class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')
        self._shutdown_requested = False
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        # Best Effort QoS for Subscription (matches sensor)
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Reliable QoS for Publisher (matches SLAM/Nav2 requirements)
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_best_effort
        )
        
        self.pub = self.create_publisher(
            LaserScan,
            'scan_reliable',
            qos_reliable
        )
        self.get_logger().info('Scan Relay Node Started: /scan (BestEffort) -> /scan_reliable (Reliable)')

    def _signal_handler(self, signum, frame):
        self._shutdown_requested = True

    def listener_callback(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanRelay()
    
    try:
        while rclpy.ok() and not node._shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

