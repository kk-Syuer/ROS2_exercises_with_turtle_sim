#!/urs/bin/env pytho3
import rclpy
from rclpy.node import Node
#Twist is the type published by /turtle1/cmd_vel
#it has 2 fields linear geometry_msgs/Vecor3 and angular geometry_msgs/Vecor3
from geometry_msgs.msg import Twist

class TurtleSubscriber(Node):
	def __init__(self):
		super().__init__('turtle_subscriber')
		self.sub = self.create_subscription(Twist, '/turtle_cmd', self.cb, 10)
		self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
		
	def cb(self, msg: Twist):
		self.pub.publish(msg)
		
def main():
	rclpy.init()
	rclpy.spin(TurtleSubscriber())	#keep node running until manually stopped
	rclpy.shutdown()

if __name__=='__main__':
	main()
