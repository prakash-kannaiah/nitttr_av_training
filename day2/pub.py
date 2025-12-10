import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class frst_cls(Node):
	def __init__(self):
		super().__init__('frst_node')
		self.publisher = self.create_publisher(String, 'myTopic', 10)
		self.calFn = self.create_timer(1.0,self.myFn)		

	def myFn(self):
		msg = String()
		msg.data = 'hello from ROS'
		self.publisher.publish(msg)
		self.get_logger().info(msg.data)
		
def main(args=None):
	rclpy.init(args=args)
	myObj = frst_cls()
	rclpy.spin(myObj)
	myObj.destroy_node()
	rclpy.shutdown()	

if __name__ == '__main__':
	main()
