import rclpy
from rclpy.node import Node

class CroppedPointCloudSubscriber(Node):

def main(args=None):
    rclpy.init(args=args)
    node = CroppedPointCloudSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
