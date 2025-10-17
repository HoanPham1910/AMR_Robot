import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion

class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        # Broadcaster để gửi transform
        self.tf_broadcaster = TransformBroadcaster(self)

        # Đăng ký subscriber /odom
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info("📡 TF Broadcaster từ /odom → /base_link đã khởi tạo!")

    def odom_callback(self, msg: Odometry):
        # Lấy dữ liệu vị trí và hướng
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        # Tạo message TransformStamped
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = pos.x
        t.transform.translation.y = pos.y
        t.transform.translation.z = 0.0
        t.transform.rotation = ori

        # Gửi transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
