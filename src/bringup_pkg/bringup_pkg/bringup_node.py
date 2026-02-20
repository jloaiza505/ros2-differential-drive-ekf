import rclpy
from rclpy.node import Node


class BringupNode(Node):
    def __init__(self):
        super().__init__('bringup_node')
        self.get_logger().info('bringup_pkg node initialized')


def main(args=None):
    rclpy.init(args=args)
    node = BringupNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
