import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from walle_movement.SurveyorSRV1MotorBoard import SurveyorSRV1MotorBoard


class WallEMovementListener(Node):

    def __init__(self):
        super().__init__('walle_movement_listener')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.driver = SurveyorSRV1MotorBoard()

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)
        if msg.linear.x > 0.0:
            self.driver.motorsMove('forward')
        elif msg.linear.x < 0.0:
            self.driver.motorsMove('backward')
        elif msg.angular.z > 0.0:
            self.driver.motorsMove('left')
        elif msg.angular.z < 0.0:
            self.driver.motorsMove('right')
        else:
            self.driver.motorsStop()



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

