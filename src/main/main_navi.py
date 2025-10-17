import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from Library.ezi_servo_controller import EziServoController
import math

class CmdVelToEzi(Node):
    def __init__(self, motor_left, motor_right):
        super().__init__('cmd_vel_to_ezi')
        self.motor_left = motor_left
        self.motor_right = motor_right

        # 🔹 Thông số robot
        self.wheel_radius = 0.05       # mét
        self.wheel_distance = 0.30     # mét

        # --- Lưu chiều hiện tại ---
        self.dir_left = 1
        self.dir_right = 1

        # --- Kích hoạt chế độ velocity ngay khi khởi động ---
        self.motor_left.move_velocity_rpm(0, direction=self.dir_left)
        self.motor_right.move_velocity_rpm(0, direction=self.dir_right)
        self.get_logger().info("🔄 Đã khởi động chế độ velocity (0 RPM)")

        # --- Đăng ký subscriber ---
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('✅ Node cmd_vel_to_ezi đang chờ lệnh /cmd_vel ...')

    def cmd_vel_callback(self, msg):
        v = msg.linear.x     # m/s
        w = msg.angular.z    # rad/s

        # --- Tính vận tốc từng bánh ---
        v_left = v - (w * self.wheel_distance / 2.0)
        v_right = v + (w * self.wheel_distance / 2.0)

        # --- Đổi sang RPM ---
        rpm_left = (v_left / (2 * math.pi * self.wheel_radius)) * 60
        rpm_right = (v_right / (2 * math.pi * self.wheel_radius)) * 60

        # --- Giới hạn tốc độ ---
        rpm_left = max(min(rpm_left, 1000), -1000)
        rpm_right = max(min(rpm_right, 1000), -1000)

        # --- Xác định chiều mới ---
        new_dir_left = 1 if rpm_left >= 0 else 0
        new_dir_right = 1 if rpm_right >= 0 else 0

        # --- Gửi lệnh tới driver ---
        # Nếu đổi chiều, dùng move_velocity_rpm để đảm bảo driver nhận chiều mới
        if new_dir_left != self.dir_left:
            self.motor_left.move_velocity_rpm(abs(rpm_left), direction=new_dir_left)
            self.dir_left = new_dir_left
        else:
            self.motor_left.velocity_override_rpm(abs(rpm_left))

        if new_dir_right != self.dir_right:
            self.motor_right.move_velocity_rpm(abs(rpm_right), direction=new_dir_right)
            self.dir_right = new_dir_right
        else:
            self.motor_right.velocity_override_rpm(abs(rpm_right))

        # --- Log theo dõi ---
        self.get_logger().info(
            f"CmdVel → v={v:.2f} m/s, w={w:.2f} rad/s | "
            f"Left={rpm_left:.1f} RPM (dir={new_dir_left}), "
            f"Right={rpm_right:.1f} RPM (dir={new_dir_right})"
        )

def main():
    # --- Kết nối EziServo qua Ethernet ---
    motor_left = EziServoController("192.168.0.2", 2001)
    motor_right = EziServoController("192.168.0.7", 2001)

    # --- Khởi động ROS2 Node ---
    rclpy.init()
    node = CmdVelToEzi(motor_left, motor_right)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Dừng chương trình")
    finally:
        # --- Dừng motor trước khi shutdown ---
        motor_left.stop()
        motor_right.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
