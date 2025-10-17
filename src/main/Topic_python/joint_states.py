import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math


class EncoderJointStatePublisher(Node):
    def __init__(self, motor_left, motor_right):
        super().__init__('encoder_joint_state_publisher')
        self.motor_left = motor_left
        self.motor_right = motor_right

        # ===== Encoder & timer =====
        self.prev_left_pos = self.motor_left.get_actual_position()
        self.prev_right_pos = self.motor_right.get_actual_position()
        self.last_time = time.time()

        # ===== Cấu hình robot =====
        self.ticks_per_rev = 10000
        self.wheel_radius = 0.075

        # ===== ROS2 Publisher =====
        self.pub_joint = self.create_publisher(JointState, '/joint_states', 10)

        # ===== Timer định kỳ =====
        self.timer = self.create_timer(1, self.publish_joint_state)

        self.get_logger().info("Node encoder_joint_state_publisher đã khởi tạo thành công!")

    # -------------------------------------------------------
    def get_encoder_positions(self):
        """Đọc vị trí encoder (tính bằng xung) từ 2 động cơ"""
        try:
            left_pos = self.motor_left.get_actual_position()
            right_pos = self.motor_right.get_actual_position()
            return left_pos, right_pos
        except Exception as e:
            self.get_logger().error(f"⚠️ Lỗi đọc encoder: {e}")
            return None, None

    # -------------------------------------------------------
    def publish_joint_state(self):
        left_ticks, right_ticks = self.get_encoder_positions()
        if left_ticks is None or right_ticks is None:
            return

        # # ===== In ra giá trị encoder để kiểm tra =====
        # self.get_logger().info(f"🔹 Encoder check | L: {left_ticks} | R: {right_ticks}")

        # ===== Tính thời gian =====
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0:
            return

        # ===== Tính góc quay hiện tại =====
        left_angle = 2 * math.pi * (abs(left_ticks) / self.ticks_per_rev)
        right_angle = 2 * math.pi * (abs(right_ticks) / self.ticks_per_rev)

        # ===== Tính delta và vận tốc =====
        delta_left_angle = 2 * math.pi * (abs(left_ticks) - self.prev_left_pos) / self.ticks_per_rev
        delta_right_angle = 2 * math.pi * (abs(right_ticks) - self.prev_right_pos) / self.ticks_per_rev

        left_vel = delta_left_angle / dt
        right_vel = delta_right_angle / dt

        # ===== Cập nhật prev encoder =====
        self.prev_left_pos = abs(left_ticks)
        self.prev_right_pos = abs(right_ticks)

        # ===== Tạo message JointState =====
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['wheel_left_joint', 'wheel_right_joint']
        msg.position = [left_angle, right_angle]
        msg.velocity = [left_vel, right_vel]
        msg.effort = []

        self.pub_joint.publish(msg)

        # self.get_logger().info(
        #     f"📊 L: {left_angle:.2f} rad ({left_vel:.2f} rad/s) | "
        #     f"R: {right_angle:.2f} rad ({right_vel:.2f} rad/s)"
        # )

    # -------------------------------------------------------
    def destroy(self):
        """Tắt servo và đóng kết nối an toàn"""
        self.get_logger().info("🛑 Đang đóng kết nối động cơ...")
        try:
            self.motor_left.set_servo_enable(False)
            self.motor_right.set_servo_enable(False)
            self.motor_left.close()
            self.motor_right.close()
        except Exception as e:
            self.get_logger().warn(f"Lỗi khi đóng kết nối: {e}")


# -------------------------------------------------------
def main(args=None):
    """Cho phép chạy độc lập để test nhanh"""
    from Library.ezi_servo_controller import EziServoController

    rclpy.init(args=args)

    motor_left = EziServoController("192.168.0.2", 2001)
    motor_right = EziServoController("192.168.0.7", 2001)
    node = EncoderJointStatePublisher(motor_left, motor_right)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🧹 Dừng node bằng Ctrl+C")
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Đã tắt node encoder_joint_state_publisher an toàn.")


if __name__ == '__main__':
    main()
