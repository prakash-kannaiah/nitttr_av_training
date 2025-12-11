import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class LidarCropper(Node):
    def __init__(self):
        super().__init__('lidar_cropper')

        # Match rosbag2 QoS (BEST_EFFORT, VOLATILE)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber to original pointcloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/sensing/lidar/top/ouster/points',
            self.callback,
            qos
        )

        # Publisher for cropped cloud
        self.publisher = self.create_publisher(PointCloud2, '/cropped_lidar', 10)

    def callback(self, msg):
        self.get_logger().info("Received a pointcloud message!")

        # Extract (x, y, z) points
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        cropped_points = []
        for x, y, z in points:
            # angle = math.degrees(math.atan2(y, x))
            angle = math.degrees(math.atan2(y, x))
            #if (0 < angle <= 180):
            if ((angle >= 150) | (angle <= -150)):
                cropped_points.append((x, y, z))

        self.get_logger().info(f"Cropped points count: {len(cropped_points)}")

        if not cropped_points:
            return

        # Define x, y, z fields
        fields = [
            PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
        ]

        header = msg.header
        cropped_cloud = pc2.create_cloud(header, fields, cropped_points)
        self.publisher.publish(cropped_cloud)


def main(args=None):
    rclpy.init(args=args)
    node = LidarCropper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
