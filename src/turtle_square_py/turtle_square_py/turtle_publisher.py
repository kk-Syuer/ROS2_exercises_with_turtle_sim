#!/urs/bin/env pytho3
import rclpy
from rclpy.node import Node
#Twist is the type published by /turtle1/cmd_vel
#it has 2 fields linear geometry_msgs/Vecor3 and angular geometry_msgs/Vecor3
from geometry_msgs.msg import Twist, Vector3

#define the movements that will form a square path
#each vector3 represents a direction of a linear movement
#for angular motion, only z is used because the turtle rotates around the z-axis
#msg.angular.z = 1.0 means rotate counterclockwise, turn left at 1 radiant per second
turtle_moves = [
	Vector3(x= 1.0, y= 0.0, z= 0.0),	#move right
	Vector3(x= 0.0, y= 1.0, z= 0.0),	#move up
	Vector3(x= -1.0, y= 0.0, z= 0.0),	#move left
	Vector3(x= 0.0, y= -1.0, z= 0.0),	#move down
	]
	
class TurtlePublisher(Node):
	def __init__(self):
		super().__init__('turtle_publisher')
		#create a publisher for the twist message on the turtle's velocity topuc
		self.publisher = self.create_publisher(
			Twist,	#message type
			'/turtle1/cmd_vel',	#topic name
			10	#queue size
		)
	
		#create a timer to call timer_callback every1 sec
		self.timer = self.create_timer(1.0, self.timer_callback)
		
		#keep track of which movement we're on 
		self.i = 0
		
	def timer_callback(self):
		#create a Twist message
		msg = Twist()
		
		#set linear velocity based on our turtle_moves list
		msg.linear = turtle_moves[self.i % 4] #%4 keeps it looping through 0-3
		
		#set andular velocity to 0 so it doesn't rotate
		msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
		
		#log which move we're publishing
		self.get_logger().info(f'Move {self.i+1}: {msg.linear}')
		
		#publish message
		self.publisher.publish(msg)
		
		#increment the index to go to the next direction
		self.i+=1
def main(args=None):
	rclpy.init(args=args)
	node = TurtlePublisher()
	rclpy.spin(node)	#keep node running until manually stopped
	node.destroy_node()
	rclpy.shutdown()

if __name__=='__main__':
	main()
