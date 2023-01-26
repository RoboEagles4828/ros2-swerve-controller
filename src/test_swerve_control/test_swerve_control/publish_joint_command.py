import rclpy
from rclpy.node import Node
from rclpy.qos import QoSReliabilityPolicy, QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
import math

from sensor_msgs.msg import JointState


class PublishJointCmd(Node):

    def __init__(self):
        super().__init__('publish_joint_commands')
        # self.qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=1)
        self.publisher_ = self.create_publisher(JointState, 'isaac_joint_commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        velocity_cmds = JointState()
        # test = JointState()
        # position_cmds = JointState()
        
        velocity_cmds.name = [
            'front_left_wheel_joint', 
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
            'front_left_axle_joint', 
            'front_right_axle_joint',
            'rear_left_axle_joint',
            'rear_right_axle_joint']
        # test.name = ['test1', 'test2']
        # position_cmds.name = []

        rad_per = 2 * math.pi
        
        velocity_cmds.velocity = [0.0, 0.0, 0.0, 0.0, rad_per, rad_per, rad_per, rad_per]
        # position_cmds.position = []

        self.publisher_.publish(velocity_cmds)
        # self.publisher_.publish(position_cmds)
        self.get_logger().info('Publishing: ...')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    node = PublishJointCmd()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()