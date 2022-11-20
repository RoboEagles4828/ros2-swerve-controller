import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3


class PublishTwistCmd(Node):

    def __init__(self):
        super().__init__('publish_twist_commands')
        self.publisher_ = self.create_publisher(Twist, 'swerve_controller/cmd_vel_unstamped', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        twist = Twist()

        twist.linear.x = 2.0
        twist.linear.y = 100.0
        twist.angular.z = 4.0
        
        self.publisher_.publish(twist)
        self.get_logger().info('Publishing: ...')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = PublishTwistCmd()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()