import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
import math
import time


class OdomPublisher(Node):
    def __init__(self, wheel_radius=0.075, wheel_base=0.65):
        super().__init__('odom_publisher')

        # === Thông số robot ===
        self.wheel_radius = wheel_radius  # (m)
        self.wheel_base = wheel_base      # khoảng cách 2 bánh (m)

        # === Dữ liệu vị trí ===
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0  # góc hướng robot (rad)

        # === Biến lưu trước đó ===
        self.prev_left = 0.0
        self.prev_right = 0.0
        self.last_time = self.get_clock().now()

        # === Sub từ /joint_states ===
        self.sub_joint = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        # === Pub ra /odom ===
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)

        self.get_logger().info("🚀 Node odom_publisher đã khởi tạo thành công!")
    
    # ---------------------------------------------------------
    def joint_callback(self, msg: JointState):
        # Lấy vị trí góc bánh xe (rad)
        left_pos = msg.position[0]
        right_pos = msg.position[1]

        # Tính delta giữa 2 lần cập nhật
        delta_left = left_pos - self.prev_left
        delta_right = right_pos - self.prev_right

        
        # Lưu lại cho lần sau
        self.prev_left = left_pos
        self.prev_right = right_pos

        # Tính trung bình quãng đường di chuyển
        delta_s = self.wheel_radius * (delta_right + delta_left) / 2.0
        delta_theta = self.wheel_radius * (delta_right - delta_left) / self.wheel_base

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0:
            return

        # Tính pose mới
        self.x += delta_s * math.cos(self.th + delta_theta / 2.0)
        self.y += delta_s * math.sin(self.th + delta_theta / 2.0)
        self.th += delta_theta

        # Tính vận tốc
        vx = delta_s / dt
        vth = delta_theta / dt

        # Chuyển sang quaternion để publish
        q = quaternion_from_euler(0, 0, self.th)

        # Tạo msg Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x 
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3]
        )

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        
        self.pub_odom.publish(odom)

        self.get_logger().info(
            f"📍 Odom → x={self.x:.2f}, y={self.y:.2f}, θ={math.degrees(self.th):.1f}° | "
            f"vx={vx:.2f} m/s, ω={vth:.2f} rad/s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🧹 Dừng node odom_publisher bằng Ctrl+C")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
