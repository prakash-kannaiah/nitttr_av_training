import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math

class CroppedPointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('cropped_pointcloud_subscriber')

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT  # Set best effort

        self.subscription = self.create_subscription(
            PointCloud2,
            '/sensing/lidar/top/pointcloud',
            self.pointcloud_callback,
            qos
        )

    def pointcloud_callback(self, msg):
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        cropped_points = []
        for x, y, z in points:
            angle = math.degrees(math.atan2(y, x)) + 90  # <-- Phase shift to match LiDAR 0° = left
            if 80 <= angle <= 100:  # Narrow front cone
                cropped_points.append((x, y, z))

        self.get_logger().info(f"Total points: {len(points)} | Cropped points: {len(cropped_points)}")


def main(args=None):
    rclpy.init(args=args)
    node = CroppedPointCloudSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
