import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from geometry_msgs.msg import Vector3, Point

from patrol_interfaces.action import PatrolCommandInterface


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def angle_normalize(a):
    # wrap to [-pi, pi]
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class PatrollingActionServer(Node):
    def __init__(self):
        super().__init__('patrolling_action_server')

        # we need pose updates while running long loops -> reentrant group + multithreaded executor
        self.cb_group = ReentrantCallbackGroup()

        self.pose = Pose()
        self.pose_ready = False
        self.create_subscription(Pose, '/turtle1/pose', self._pose_cb, 10,
                                 callback_group=self.cb_group)

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.action_server = ActionServer(
            self,
            PatrolCommandInterface,
            'command_turtle',
            execute_callback=self.execute_cb,
            callback_group=self.cb_group,
        )

    def _pose_cb(self, msg: Pose):
        self.pose = msg
        self.pose_ready = True

    # === tiny helpers ===
    def _bearing_to(self, target: Point) -> float:
        return math.atan2(target.y - self.pose.y, target.x - self.pose.x)

    def _dist_to(self, target: Point) -> float:
        dx, dy = target.x - self.pose.x, target.y - self.pose.y
        return math.hypot(dx, dy)

    def _publish(self, lin_x: float, ang_z: float):
        msg = Twist()
        msg.linear = Vector3(x=lin_x, y=0.0, z=0.0)
        msg.angular = Vector3(x=0.0, y=0.0, z=ang_z)
        self.cmd_pub.publish(msg)

    # === main action logic ===
    async def execute_cb(self, goal_handle):
        self.get_logger().info('Patrol goal received')
        feedback = PatrolCommandInterface.Feedback()
        result = PatrolCommandInterface.Result()

        # wait for first pose
        while not self.pose_ready:
            rclpy.sleep(0.05)

        targets = goal_handle.request.targets
        if not targets:
            result.done = True
            return result

        # tuning knobs (gentle values)
        ANG_KP = 2.0     # turn speed gain
        LIN_KP = 1.5     # forward speed gain
        ANG_MAX = 2.0
        LIN_MAX = 2.0
        ANG_TOL = 0.05   # rad ~3 deg
        POS_TOL = 0.1    # meters in turtlesim space

        for i, tgt in enumerate(targets):
            if goal_handle.is_cancel_requested:
                self._publish(0.0, 0.0)
                goal_handle.canceled()
                result.done = False
                return result

            # 1) rotate toward target
            while rclpy.ok():
                desired = self._bearing_to(tgt)
                err = angle_normalize(desired - self.pose.theta)
                if abs(err) < ANG_TOL:
                    break
                ang = clamp(ANG_KP * err, -ANG_MAX, ANG_MAX)
                self._publish(0.0, ang)
                time.sleep(0.02)  # ~50 Hz

            # 2) drive straight to target (with tiny heading correction)
            while rclpy.ok():
                dist = self._dist_to(tgt)
                if dist < POS_TOL:
                    break
                desired = self._bearing_to(tgt)
                ang_err = angle_normalize(desired - self.pose.theta)
                lin = clamp(LIN_KP * dist, 0.0, LIN_MAX)
                ang = clamp(ANG_KP * ang_err, -ANG_MAX, ANG_MAX)
                self._publish(lin, ang)
                time.sleep(0.02)

            # reached waypoint i
            feedback.current_index = i
            goal_handle.publish_feedback(feedback)

        # stop and succeed
        self._publish(0.0, 0.0)
        goal_handle.succeed()
        result.done = True
        return result


def main():
    rclpy.init()
    node = PatrollingActionServer()
    exec_ = MultiThreadedExecutor()   # lets pose callback run while loops run
    exec_.add_node(node)
    exec_.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
