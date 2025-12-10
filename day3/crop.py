import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from rclpy.qos import QoSProfile, QoSReliabilityPolicy


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

def main(args=None):
    rclpy.init(args=args)
    node = CroppedPointCloudSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
