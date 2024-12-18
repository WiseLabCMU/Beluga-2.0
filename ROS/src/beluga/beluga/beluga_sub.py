import rclpy
from rclpy.node import Node

from beluga_messages.msg import BelugaNeighbors


class BelugaSubscriber(Node):
    def __init__(self, sub_topic: str = "neighbors_list", history_depth: int = 10):
        super().__init__('beluga_subscriber')
        if not isinstance(sub_topic, str):
            raise TypeError('Publisher topic needs to be a string')
        if not isinstance(history_depth, int):
            raise TypeError('Publisher history depth must be an integer')
        self.subscription = self.create_subscription(
            BelugaNeighbors,
            sub_topic,
            self.listener_callback,
            history_depth)

    def listener_callback(self, msg):
        self.get_logger().info('I heard:\n' + '\n'.join(f'{{"ID": {x.id}, "RANGE": {x.distance}, "RSSI": {x.rssi}, "TIMESTAMP": {x.timestamp}, "EXCHANGE": {x.exchange}}}' for x in msg.neighbors))


def main(args=None):
    rclpy.init(args=args)
    beluga_sub = BelugaSubscriber()
    rclpy.spin(beluga_sub)
    beluga_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
