import rclpy
import rclpy.node
import rclpy.parameter

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')

        # declare with default
        self.declare_parameter('my_parameter', 'world')

        # tick every second
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
        self.get_logger().info('Hello %s!' % my_param)

        # (prof’s example) reset param to "world" every tick
        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'world'
        )
        self.set_parameters([my_new_param])

def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
