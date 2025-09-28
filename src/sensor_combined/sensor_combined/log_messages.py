
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import LogMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PX4LogListener(Node):
    def __init__(self):
        super().__init__('px4_log_listener')

        orb_queue_length = 4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=orb_queue_length
        )

        self.subscription = self.create_subscription(
            LogMessage,
            '/fmu/out/log_message',
            self.log_cb,
            qos_profile
        )

        self.get_logger().info("Subscribed to PX4 log messages.")

    def log_cb(self, msg: LogMessage):
        severity_levels = ["EMERG", "ALERT", "CRIT", "ERR", "WARNING", "NOTICE", "INFO", "DEBUG"]
        severity_str = severity_levels[msg.severity] if msg.severity < len(severity_levels) else "UNKNOWN"

       
        log_text = bytes(msg.text).decode('utf-8', errors='ignore').rstrip('\x00')

        self.get_logger().info(f"[PX4 LOG] [{severity_str}] {log_text}")

def main(args=None):
    rclpy.init(args=args)
    node = PX4LogListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

