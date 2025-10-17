import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback = Fibonacci.Feedback()
        feedback.partial_sequence = [0, 1]

        # build sequence and publish feedback
        for i in range(1, goal_handle.request.order):
            feedback.partial_sequence.append(
                feedback.partial_sequence[i] + feedback.partial_sequence[i - 1]
            )
            goal_handle.publish_feedback(feedback)
            time.sleep(0.5)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback.partial_sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
