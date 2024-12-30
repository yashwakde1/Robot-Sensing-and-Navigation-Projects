#! /usr/bin/python3
import rclpy
from rclpy.node import Node

#from std_msgs.msg import String
from tutorial_interfaces.msg import Sphere

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Sphere, 'TOPIC', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Sphere()
        msg.center.x=1.2
        msg.center.y=1.3
        msg.center.z=1.4
        msg.radius=2.0
        self.get_logger().info(f"radius={msg.radius}")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
