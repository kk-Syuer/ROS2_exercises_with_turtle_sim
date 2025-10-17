#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def norm_angle(a):
    while a > math.pi:  a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

class WallSquareFollower(Node):
    def __init__(self):
        super().__init__('wall_square_follower')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)

        self.forward_speed = 1.5
        self.turn_speed    = 2.0
        self.edge_margin   = 0.8
        self.escape_seconds = 0.40     # <- NEW: move away from wall after turn

        self.heading_targets = [0.0, math.pi/2, math.pi, -math.pi/2]  # E,N,W,S
        self.heading_index = 0
        self.state = 'forward'         # 'forward' | 'turn' | 'escape'
        self.pose = None
        self.phase_start = None

        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz

    def on_pose(self, msg: Pose):
        self.pose = msg
        if self.phase_start is None:
            self.phase_start = self.get_clock().now().nanoseconds / 1e9

    def near_edge(self, p: Pose) -> bool:
        return (p.x >= 11.0 - self.edge_margin or
                p.x <= self.edge_margin or
                p.y >= 11.0 - self.edge_margin or
                p.y <= self.edge_margin)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def loop(self):
        if self.pose is None:
            return
        t = self.now()
        cmd = Twist()

        if self.state == 'forward':
            cmd.linear.x = self.forward_speed
            if self.near_edge(self.pose):
                # start turning to next cardinal heading
                self.state = 'turn'
                self.heading_index = (self.heading_index + 1) % 4
                self.phase_start = t
                self.get_logger().info('Corner reached → turning')

        elif self.state == 'turn':
            target = self.heading_targets[self.heading_index]
            err = norm_angle(target - self.pose.theta)
            cmd.angular.z = self.turn_speed if err > 0 else -self.turn_speed
            if abs(err) < 0.04:  # ~2.3°
                # finished turn → escape straight a little bit
                self.state = 'escape'
                self.phase_start = t
                self.get_logger().info('Heading aligned → escape forward')

        elif self.state == 'escape':
            # move forward without edge check to leave the wall area
            cmd.linear.x = self.forward_speed
            if t - self.phase_start >= self.escape_seconds:
                self.state = 'forward'
                self.get_logger().info('Escape done → normal forward')

        self.pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(WallSquareFollower())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

