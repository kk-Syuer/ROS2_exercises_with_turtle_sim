#!/urs/bin/env pytho3
import rclpy
from rclpy.node import Node
#Twist is the type published by /turtle1/cmd_vel
#it has 2 fields linear geometry_msgs/Vecor3 and angular geometry_msgs/Vecor3
from geometry_msgs.msg import Twist, Vector3

class MoveForward(Node):
	def __init__(self):
		super().__init__("move_forward")
		self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
		self.timer = self.create_timer(0.1, self.tick)
	def tick(self):
		msg = Twist()
		msg.linear.x=1.0 # this is what turtle sim uses to go forward
		msg.angular.z=0.0
		self.pub.publish(msg)
def main():
	rclpy.init()
	rclpy.spin(MoveForward())
	rclpy.shutdown()

if __name__=='__main__':
	main()
