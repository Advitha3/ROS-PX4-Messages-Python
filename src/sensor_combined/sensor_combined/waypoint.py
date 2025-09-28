#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    VehicleCommand,
    TrajectorySetpoint,
    OffboardControlMode,
    VehicleStatus,
    VehicleGlobalPosition,
    VehicleOdometry,
    HomePosition,
)
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
VEHICLE_CMD_NAV_VTOL_TAKEOFF = 84
VEHICLE_CMD_DO_VTOL_TRANSITION = 3001
VEHICLE_CMD_DO_FIGUREEIGHT = 35
VEHICLE_CMD_DO_SET_HOME = 179


class PX4FigureEightMission(Node):
    def __init__(self):
        super().__init__('px4_figure_eight_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)

        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, qos)
        self.create_subscription(VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.global_pos_cb, qos)
        self.create_subscription(HomePosition, '/fmu/out/home_position', self.home_cb, qos)

        self.timer = self.create_timer(0.1, self.timer_cb)

        self.current_position = [0.0, 0.0, 0.0]
        self.vehicle_status = VehicleStatus()
        self.vehicle_global_position = VehicleGlobalPosition()

        self.home_sent = False
        self.home_valid = False
        self.armed = False
        self.taken_off = False
        self.transitioned = False
        self.figure8_sent = False

        self.home_position = [0.0, 0.0, 0.0]
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.home_alt = 0.0

        self.takeoff_alt = 40.0
        self.waypoint = [200.0, 0.0, -70.0]

    def odom_cb(self, msg):
        self.current_position = list(msg.position)

    def status_cb(self, msg):
        self.vehicle_status = msg

    def global_pos_cb(self, msg):
        self.vehicle_global_position = msg

    def home_cb(self, msg):
        if msg.valid_lpos:
            self.home_position = [msg.x, msg.y, msg.z]
            if not self.home_valid:
                self.home_valid = True
                self.home_lat = msg.lat  
                self.home_lon = msg.lon  
                self.home_alt = msg.alt 
                self.get_logger().info(f"Home set (local): {self.home_position}")
                self.get_logger().info(f"Home GPS: lat={self.home_lat}, lon={self.home_lon}, alt={self.home_alt}")

    def publish_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = [
            self.home_position[0] + self.waypoint[0],
            self.home_position[1] + self.waypoint[1],
            self.home_position[2] + self.waypoint[2]
        ]
        msg.yaw = 0.0
        self.setpoint_pub.publish(msg)

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.mode_pub.publish(msg)

    def send_cmd(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def timer_cb(self):
        if not self.home_sent:
            self.send_cmd(VEHICLE_CMD_DO_SET_HOME, param1=1.0)
            self.get_logger().info("Set home to current position")
            self.home_sent = True
            return

        if self.home_sent and not self.home_valid:
            self.get_logger().info("Waiting for home confirmation...")
            return

        if not self.armed and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            self.send_cmd(VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.get_logger().info("Arming...")
            self.armed = True
            return

        if self.armed and not self.taken_off and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.send_cmd(
                VEHICLE_CMD_NAV_VTOL_TAKEOFF,
                param5=self.vehicle_global_position.lat,
                param6=self.vehicle_global_position.lon,
                param7=self.takeoff_alt + self.vehicle_global_position.alt
            )
            self.get_logger().info("Takeoff command sent")
            self.taken_off = True
            return

        if self.taken_off and not self.transitioned and self.vehicle_global_position.alt > self.takeoff_alt - 5:
            self.send_cmd(VEHICLE_CMD_DO_VTOL_TRANSITION, param1=4.0)
            self.get_logger().info("Transition to fixed-wing")
            self.transitioned = True
            return

        self.publish_offboard_mode()
        self.publish_setpoint()

        if (self.transitioned and not self.figure8_sent and
            self.vehicle_status.vehicle_type == VehicleStatus.VEHICLE_TYPE_FIXED_WING):
            self.send_cmd(
                VEHICLE_CMD_DO_FIGUREEIGHT,
                param1=150.0,
                param2=50.0,
                param3=5.0,
                param5=self.home_lat,
                param6=self.home_lon,
                param7=self.home_alt + 150.0
            )
            self.figure8_sent = True
            self.get_logger().info("Figure-8 command sent")


def main(args=None):
    rclpy.init(args=args)
    node = PX4FigureEightMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
