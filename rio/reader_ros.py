import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import Joy

class OrinSubscriber(Node):
    
    def __init__(self):
        super().__init__('orin_subscriber')

        qos_profile = qos.QoSProfile(depth=1, reliability=qos.ReliabilityPolicy.BEST_EFFORT, durability=qos.DurabilityPolicy.VOLATILE)
        qos_profile_2 = qos.QoSProfile(depth=1, reliability=qos.ReliabilityPolicy.SYSTEM_DEFAULT, durability=qos.DurabilityPolicy.SYSTEM_DEFAULT)

        self.subscription = self.create_subscription(Joy, r'joystick_data', self.listener_callback, qos_profile_2)

    def listener_callback(self, msg):
        self.get_logger().info('Recieved: "%s"' % msg.data)

def main(args=None):
    print('Init')
    rclpy.init(args=args)

    orin_subscriber = OrinSubscriber()

    rclpy.spin(orin_subscriber)

    orin_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()