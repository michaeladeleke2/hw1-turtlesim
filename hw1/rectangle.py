import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RectangleNode(Node):
    def __init__(self):
        super().__init__('rectangle_node')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.02, self.tick)

        self.linear_speed = 2.0
        self.turn_speed = 1.5708  # ~90°/sec
        self.side_time = 2.0
        self.turn_time = 1.0

        # sequence: forward, turn, repeat 4x
        self.actions = []
        for _ in range(4):
            self.actions.append(('forward', self.side_time))
            self.actions.append(('turn', self.turn_time))

        self.current = 0
        self.remaining = self.actions[0][1]

    def tick(self):
        if self.current >= len(self.actions):
            self.pub.publish(Twist())
            self.get_logger().info('Rectangle complete. Stopping.')
            rclpy.shutdown()
            return

        phase, _ = self.actions[self.current]
        dt = self.timer.timer_period_ns / 1e9
        self.remaining -= dt

        cmd = Twist()
        if phase == 'forward':
            cmd.linear.x = self.linear_speed
        elif phase == 'turn':
            cmd.angular.z = self.turn_speed

        self.pub.publish(cmd)

        if self.remaining <= 0.0:
            self.current += 1
            if self.current < len(self.actions):
                self.remaining = self.actions[self.current][1]


def main():
    rclpy.init()
    node = RectangleNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()