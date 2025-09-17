import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DiamondNode(Node):
    def __init__(self):
        super().__init__('diamond_node')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.02, self.tick)

        self.linear_speed = 2.0
        self.turn_speed = 1.5708  # ~90°/sec
        self.turn45_speed = 0.7854  # ~45°/sec

        self.side_time = 1.8
        self.turn90_time = 1.0
        self.turn45_time = 1.0

        # sequence: rotate +45°, then draw square
        self.actions = [('turn45', self.turn45_time)]
        for _ in range(4):
            self.actions.append(('forward', self.side_time))
            self.actions.append(('turn90', self.turn90_time))

        self.current = 0
        self.remaining = self.actions[0][1]

    def tick(self):
        if self.current >= len(self.actions):
            self.pub.publish(Twist())
            self.get_logger().info('Diamond complete. Stopping.')
            rclpy.shutdown()
            return

        phase, _ = self.actions[self.current]
        dt = self.timer.timer_period_ns / 1e9
        self.remaining -= dt

        cmd = Twist()
        if phase == 'forward':
            cmd.linear.x = self.linear_speed
        elif phase == 'turn90':
            cmd.angular.z = self.turn_speed
        elif phase == 'turn45':
            cmd.angular.z = self.turn45_speed

        self.pub.publish(cmd)

        if self.remaining <= 0.0:
            self.current += 1
            if self.current < len(self.actions):
                self.remaining = self.actions[self.current][1]


def main():
    rclpy.init()
    node = DiamondNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()