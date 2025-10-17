import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from Library.ezi_servo_controller import EziServoController
from Topic_python.cmd_vel import ROS2Publisher
from Topic_python.joint_states import EncoderJointStatePublisher
from Topic_python.odom import OdomPublisher
from Topic_python.tf_broadcaster import OdomTFBroadcaster

from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
# ==================== ODOM SUBSCRIBER ====================
class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.get_logger().info("Node odom_subscriber đã khởi tạo!")

    def odom_callback(self, msg):
        # --- Lấy vị trí ---
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # --- Lấy góc yaw ---
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # --- Lấy vận tốc ---
        vx = msg.twist.twist.linear.x
        vth = msg.twist.twist.angular.z

        # --- Log ---
        self.get_logger().info(
            f"📍 Odom: x={x:.2f} m, y={y:.2f} m, yaw={math.degrees(yaw):.1f}° | "
            f"vx={vx:.2f} m/s, ω={vth:.2f} rad/s"
        )


def read_velocity_loop(motor_left, motor_right, stop_event):
    while not stop_event.is_set():
        try:
            vl = motor_left.get_actual_velocity_rpm()
            vr = motor_right.get_actual_velocity_rpm()
            print(f"Left={vl:.1f} RPM | Right={vr:.1f} RPM")
        except Exception as e:
            print(f"Lỗi đọc tốc độ: {e}")
        time.sleep(1)


# ==================== CHẾ ĐỘ VELOCITY ====================
def run_velocity_mode(motor_left: EziServoController, motor_right: EziServoController):
    rclpy.init()

    ros_cmd_node = ROS2Publisher(radius=0.065)
    ros_encoder_node = EncoderJointStatePublisher(motor_left, motor_right)
    ros_odom_node = OdomPublisher()
    ros_tf_node = OdomTFBroadcaster()
    ros_odom_sub_node = OdomSubscriber()

    executor = MultiThreadedExecutor()
    executor.add_node(ros_cmd_node)
    executor.add_node(ros_encoder_node)
    executor.add_node(ros_odom_node)
    executor.add_node(ros_tf_node)
    executor.add_node(ros_odom_sub_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # === BẬT SERVO AN TOÀN ===
    motor_left.set_servo_enable(True)
    motor_right.set_servo_enable(True)
    motor_left.stop()
    motor_right.stop()
    motor_left.clear_position()
    motor_right.clear_position()
    ros_cmd_node.rpm_left = 0.0
    ros_cmd_node.rpm_right = 0.0

    # === KHỞI ĐỘNG THREAD ĐỌC TỐC ĐỘ ===
    stop_event = threading.Event()
    reader_thread = threading.Thread(target=read_velocity_loop, args=(motor_left, motor_right, stop_event))
    reader_thread.start()

    prev_rpm_left = 0.0
    prev_rpm_right = 0.0
    prev_dir_left = None
    prev_dir_right = None

    try:
        # Chờ đến khi có dữ liệu thực sự từ /cmd_vel
        while ros_cmd_node.rpm_left == 0.0 and ros_cmd_node.rpm_right == 0.0:
            print("⏳ Đang chờ dữ liệu từ /cmd_vel ...")
            time.sleep(0.2)

        # === VÒNG LẶP CHÍNH ===
        while True:
            teleop_status = ros_cmd_node.check_teleop_timeout(1.0, 5.0)

            if teleop_status == "pause":
                motor_left.stop()
                motor_right.stop()
                ros_cmd_node.rpm_left = 0.0
                ros_cmd_node.rpm_right = 0.0
                time.sleep(0.05)
                continue

            elif teleop_status == "lost":
                print(" Teleop node bị ngắt hoàn toàn → shutdown ROS!")
                break

            rpm_left = ros_cmd_node.rpm_left
            rpm_right = ros_cmd_node.rpm_right

            # Nếu cả hai bánh đều gần 0 → dừng
            if abs(rpm_left) < 0.1 and abs(rpm_right) < 0.1:
                motor_left.stop()
                motor_right.stop()
                time.sleep(0.05)
                continue

            # Tính hướng quay
            dir_left = 0 if rpm_left >= 0 else 1
            dir_right = 0 if rpm_right >= 0 else 1

            # Motor trái
            if prev_dir_left != dir_left or abs(prev_rpm_left - rpm_left) > 1e-2:
                motor_left.move_velocity_rpm(abs(rpm_left), direction=dir_left)
            else:
                motor_left.velocity_override_rpm(abs(rpm_left))

            # Motor phải
            if prev_dir_right != dir_right or abs(prev_rpm_right - rpm_right) > 1e-2:
                motor_right.move_velocity_rpm(abs(rpm_right), direction=dir_right)
            else:
                motor_right.velocity_override_rpm(abs(rpm_right))

            # Lưu lại trạng thái cũ
            prev_dir_left = dir_left
            prev_dir_right = dir_right
            prev_rpm_left = rpm_left
            prev_rpm_right = rpm_right

            time.sleep(0.05)
            print(f" Left={rpm_left:.1f} RPM | Right={rpm_right:.1f} RPM")

    except KeyboardInterrupt:
        print("\n Dừng bằng Ctrl+C")

    finally:
        stop_event.set()
        reader_thread.join()
        motor_left.stop()
        motor_right.stop()
        motor_left.set_servo_enable(True)
        motor_right.set_servo_enable(True)
        motor_left.clear_position()
        motor_right.clear_position()
        ros_cmd_node.rpm_left = 0.0
        ros_cmd_node.rpm_right = 0.0
        motor_left.close()
        motor_right.close()
        executor.shutdown()
        rclpy.shutdown()
        print(" Đã dừng ROS2 và tắt servo.")


# ==================== MAIN ==================== #
def main():
    print("Chọn chế độ chạy:")
    print("1 - Position (quay theo góc)")
    print("2 - Velocity (quay theo tốc độ, publish /cmd_vel)")
    choice = input("Nhập 1 hoặc 2: ").strip()

    motor_left = EziServoController("192.168.0.2", 2002)
    motor_right = EziServoController("192.168.0.7", 2002)

    if choice == "2":
        run_velocity_mode(motor_left, motor_right)
    else:
        print("⚠️ Hiện chỉ bật chế độ velocity!")


if __name__ == "__main__":
    main()
