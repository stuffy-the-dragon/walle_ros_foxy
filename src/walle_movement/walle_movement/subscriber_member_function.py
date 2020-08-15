import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class WallEMovementListener(Node):

    def __init__(self):
        super().__init__('walle_movement_listener')
        self.subscription = self.create_subscription(
            Twist,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    walle_listener = WallEMovementListener()

    rclpy.spin(walle_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    walle_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

