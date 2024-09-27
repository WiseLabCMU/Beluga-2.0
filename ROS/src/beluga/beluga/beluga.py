import rclpy
from rclpy.node import Node
import typing

from beluga_messages.msg import BelugaNeighbor, BelugaNeighbors


class BelugaPublisherService(Node):
    def __init__(self, pub_topic: str = 'neighbors_list', pub_history_depth: int = 10, period: typing.Union[float, int] = 1, service_topic: str = 'at_commands'):
        super().__init__('beluga')
        if not isinstance(pub_topic, str):
            raise TypeError('Publisher topic needs to be a string')
        if not isinstance(pub_history_depth, int):
            raise TypeError('Publisher history depth must be an integer')
        if not isinstance(period, typing.Union[int, float]):
            raise TypeError('Period needs to be an integer or float')
        self.publisher_ = self.create_publisher(BelugaNeighbors, pub_topic, pub_history_depth)
        self.timer = self.create_timer(period, self.publish_neighbors)
        # TODO: Service
        return

    def publish_neighbors(self):
        # TODO: get neighbors list
        neighbors_dict = {1: {'ID': 1, 'RANGE': 0.5761, 'RSSI': -57, 'TIMESTAMP': 5761},
                     2: {'ID': 2, 'RANGE': 1.7621, 'RSSI': -61, 'TIMESTAMP': 5790},
                     3: {'ID': 3, 'RANGE': 5.7854, 'RSSI': -72, 'TIMESTAMP': 5004}}
        if neighbors_dict:
            neighbors_list = []
            for x in list(neighbors_dict.values()):
                neighbor = BelugaNeighbor()
                neighbor.id = x['ID']
                neighbor.distance = x['RANGE']
                neighbor.rssi = x['RSSI']
                neighbor.timestamp = x['TIMESTAMP']
                neighbors_list.append(neighbor)
            neighbor_msg = BelugaNeighbors()
            neighbor_msg.neighbors = neighbors_list
            self.publisher_.publish(neighbor_msg)
            self.get_logger().info("Publishing: " + '\n'.join(f'{{"ID": {x.id}, "RANGE": {x.distance}, "RSSI": {x.rssi}, "TIMESTAMP": {x.timestamp}}}' for x in neighbors_list))


def main(args=None):
    rclpy.init(args=args)
    beluga_pub_serv = BelugaPublisherService()
    rclpy.spin(beluga_pub_serv)
    beluga_pub_serv.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
