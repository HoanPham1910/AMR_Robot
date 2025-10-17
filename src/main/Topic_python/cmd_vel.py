import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ROS2Publisher(Node):
    def __init__(self, radius=0.075, wheel_distance=0.65):
        super().__init__('cmd_vel_listener')
        self.radius = radius
        self.wheel_distance = wheel_distance

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.rpm_left = 0.0
        self.rpm_right = 0.0

        self.last_cmd_time = time.time()
        self.teleop_connected = False
        self.teleop_alive = True  # Trạng thái teleop node

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        v_left = v - (w * self.wheel_distance / 2.0)
        v_right = v + (w * self.wheel_distance / 2.0)
        self.rpm_left = (v_left / (2 * math.pi * self.radius)) * 60.0
        self.rpm_right = (v_right / (2 * math.pi * self.radius)) * 60.0

        self.linear_x = v
        self.angular_z = w
        self.last_cmd_time = time.time()

        if not self.teleop_connected:
            self.rpm_left = 0.0
            self.rpm_right = 0.0
            self.teleop_connected = True
            self.get_logger().info("✅ Teleop reconnected")

    def check_teleop_timeout(self, stop_timeout=1.0, lost_timeout=5.0):
        """
        Kiểm tra mất tín hiệu:
        - > stop_timeout: dừng tạm thời (thả phím)
        - > lost_timeout: mất hẳn teleop node
        """
        now = time.time()
        dt = now - self.last_cmd_time

        if dt > lost_timeout:
            if self.teleop_alive:
                self.teleop_alive = False
                self.get_logger().error("❌ Teleop node lost (>5s)!")
            return "lost"

        elif dt > stop_timeout:
            if self.teleop_connected:
                self.get_logger().warn("⚠️ Tạm ngừng lệnh teleop (no cmd >1s)")
                self.teleop_connected = False
            return "pause"

        else:
            self.teleop_alive = True
            self.teleop_connected = True
            return "ok"
