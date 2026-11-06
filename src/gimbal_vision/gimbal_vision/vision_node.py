"""Vision node (ARCHITECTURE.md 4.4).

Computes the orientation error e_R = Log(cR_Cd)^v from camera images and
publishes it with the image timestamp. Event-driven: runs once per image.
"""
import rclpy
from rclpy.node import Node


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision')
        # TODO: subscribe to the camera driver
        self.get_logger().info('vision started')

    def on_image(self, msg):
        # TODO: estimate cR_Cd, compute e_R, publish with msg.header.stamp
        pass


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
