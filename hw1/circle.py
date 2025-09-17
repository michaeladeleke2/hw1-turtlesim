import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CircleNode(Node):
    def __init__(self):
        super().__init__('circle_node')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz
        self.start = self.get_clock().now()
        # choose angular speed and compute duration for ~1 full circle
        self.angular = 1.0
        self.linear = 1.5
        self.duration = 2.0 * math.pi / self.angular  # ~6.28s

    def tick(self):
        now = self.get_clock().now()
        elapsed = (now - self.start).nanoseconds / 1e9
        msg = Twist()
        if elapsed < self.duration:
            msg.linear.x = self.linear
            msg.angular.z = self.angular
            self.pub.publish(msg)
        else:
            self.pub.publish(Twist())  # stop
            self.get_logger().info('Circle complete. Stopping.')
            rclpy.shutdown()


def main():
    rclpy.init()
    node = CircleNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()