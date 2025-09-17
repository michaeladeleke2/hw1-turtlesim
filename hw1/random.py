import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class RandomNode(Node):
    def __init__(self):
        super().__init__('random_node')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

        self.pose = None
        self.t = 0.0
        self.duration = 12.0  # run ~12s then stop
        self.linear_speed = 2.0
        self.turn_speed = 1.6

    def on_pose(self, msg: Pose):
        self.pose = msg

    def tick(self):
        dt = self.timer.timer_period_ns / 1e9
        self.t += dt
        if self.t >= self.duration:
            self.pub.publish(Twist())
            self.get_logger().info('Random pattern done. Stopping.')
            rclpy.shutdown()
            return

        cmd = Twist()
        cmd.linear.x = self.linear_speed

        # 5% chance each tick to random turn
        if random.random() < 0.05:
            cmd.angular.z = random.uniform(-self.turn_speed, self.turn_speed)

        # keep inside bounds
        if self.pose:
            x, y = self.pose.x, self.pose.y
            margin = 1.0
            if x < margin:
                cmd.angular.z = abs(self.turn_speed)
            elif x > 11.0 - margin:
                cmd.angular.z = -abs(self.turn_speed)
            if y < margin:
                cmd.angular.z = abs(self.turn_speed)
            elif y > 11.0 - margin:
                cmd.angular.z = -abs(self.turn_speed)

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = RandomNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()