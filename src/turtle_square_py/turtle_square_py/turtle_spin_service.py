import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from tutorial_interfaces.srv import Spin   # our custom service


class TurtleSpinService(Node):
    def __init__(self):
        super().__init__('turtle_spin_service')
        # Publisher that actually moves the turtle
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Service server: start spinning when called
        self.srv = self.create_service(Spin, 'turtle_spin', self.spin_callback)

        # Optional timer used to stop after a while (set in callback)
        self.stop_timer = None

    def spin_callback(self, request, response):
        # Direction: +1 = CCW, -1 = CW
        d = float(request.dir)

        move = Twist()
        move.linear = Vector3(x=0.0, y=0.0, z=0.0)
        move.angular = Vector3(x=0.0, y=0.0, z=5.0 * d)  # rad/s

        self.pub.publish(move)
        self.get_logger().info(f"Spin request received. dir={d}")

        # OPTIONAL: spin only for 2 seconds, then stop
        if self.stop_timer:
            self.stop_timer.cancel()
        self.stop_timer = self.create_timer(2.0, self._stop_spin_once)

        response.res = "Spinning!"
        return response

    def _stop_spin_once(self):
        self.stop_timer.cancel()
        stop = Twist()  # all zeros → stop
        self.pub.publish(stop)
        self.get_logger().info("Spin stopped.")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpinService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
