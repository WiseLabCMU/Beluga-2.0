import rclpy
from rclpy.node import Node
import typing
from beluga.beluga_serial import BelugaSerial

from beluga_messages.msg import BelugaNeighbor, BelugaNeighbors
from beluga_messages.srv import BelugaATCommand


class BelugaPublisherService(Node):
    def __init__(self, pub_topic: str = 'neighbors_list', pub_history_depth: int = 10, period: typing.Union[float, int] = 1, service_topic: str = 'at_commands', dummy_data_mode: bool = False):
        super().__init__('beluga')
        if not isinstance(pub_topic, str):
            raise TypeError('Publisher topic needs to be a string')
        if not isinstance(pub_history_depth, int):
            raise TypeError('Publisher history depth must be an integer')
        if not isinstance(period, typing.Union[int, float]):
            raise TypeError('Period needs to be an integer or float')
        if not isinstance(service_topic, str):
            raise TypeError('Service topic must be a string')
        self.publisher_ = self.create_publisher(BelugaNeighbors, pub_topic, pub_history_depth)
        self.timer = self.create_timer(period, self.publish_neighbors)
        self.srv = self.create_service(BelugaATCommand, service_topic, self.at_command)
        self.dummy_data = dummy_data_mode
        self.serial: typing.Optional[BelugaSerial] = None
        if not self.dummy_data:
            self.serial: BelugaSerial = BelugaSerial()
        return

    def publish_neighbors(self):
        if self.dummy_data:
            neighbors_dict = {1: {'ID': 1, 'RANGE': 0.5761, 'RSSI': -57, 'TIMESTAMP': 5761},
                        2: {'ID': 2, 'RANGE': 1.7621, 'RSSI': -61, 'TIMESTAMP': 5790},
                        3: {'ID': 3, 'RANGE': 5.7854, 'RSSI': -72, 'TIMESTAMP': 5004}}
        else:
            neighbors_dict = self.serial.get_neighbors_list()
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
            self.get_logger().info("Publishing:\n" + '\n'.join(f'{{"ID": {x.id}, "RANGE": {x.distance}, "RSSI": {x.rssi}, "TIMESTAMP": {x.timestamp}}}' for x in neighbors_list))

    def at_command(self, request, response):
        if not self.dummy_data:
            try:
                arg = int(request.arg)
            except ValueError:
                arg = None
            match request.at_command:
                case BelugaATCommand.Request.AT_COMMAND_STARTUWB:
                    response.response = self.serial.start_uwb()
                case BelugaATCommand.Request.AT_COMMAND_STOPUWB:
                    response.response = self.serial.stop_uwb()
                case BelugaATCommand.Request.AT_COMMAND_STARTBLE:
                    response.response = self.serial.start_ble()
                case BelugaATCommand.Request.AT_COMMAND_STOPBLE:
                    response.response = self.serial.stop_ble()
                case BelugaATCommand.Request.AT_COMMAND_ID:
                    response.response = self.serial.id(arg)
                case BelugaATCommand.Request.AT_COMMAND_BOOTMODE:
                    response.response = self.serial.bootmode(arg)
                case BelugaATCommand.Request.AT_COMMAND_RATE:
                    response.response = self.serial.rate(arg)
                case BelugaATCommand.Request.AT_COMMAND_CHANNEL:
                    response.response = self.serial.channel(arg)
                case BelugaATCommand.Request.AT_COMMAND_RESET:
                    response.response = self.serial.reset()
                case BelugaATCommand.Request.AT_COMMAND_TIMEOUT:
                    response.response = self.serial.timeout(arg)
                case BelugaATCommand.Request.AT_COMMAND_TXPOWER:
                    response.response = self.serial.tx_power(arg)
                case BelugaATCommand.Request.AT_COMMAND_STREAMMODE:
                    response.response = self.serial.stream_mode(arg)
                case BelugaATCommand.Request.AT_COMMAND_TWRMODE:
                    response.response = self.serial.twr_mode(arg)
                case BelugaATCommand.Request.AT_COMMAND_LEDMODE:
                    response.response = self.serial.led_mode(arg)
                case BelugaATCommand.Request.AT_COMMAND_REBOOT:
                    response.response = self.serial.reboot()
                case BelugaATCommand.Request.AT_COMMAND_PWRAMP:
                    response.response = self.serial.pwr_amp(arg)
                case BelugaATCommand.Request.AT_COMMAND_ANTENNA:
                    response.response = self.serial.antenna(arg)
                case BelugaATCommand.Request.AT_COMMAND_TIME:
                    response.response = self.serial.time()
                case BelugaATCommand.Request.AT_COMMAND_DEEPSLEEP:
                    response.response = self.serial.deepsleep()
                case BelugaATCommand.Request.AT_COMMAND_DATARATE:
                    response.response = self.serial.datarate(arg)
                case BelugaATCommand.Request.AT_COMMAND_PREAMBLE:
                    response.response = self.serial.preamble(arg)
                case BelugaATCommand.Request.AT_COMMAND_PULSERATE:
                    response.response = self.serial.pulserate(arg)
                case _:
                    response.response = 'Invalid AT command'
        else:
            response.response = 'AT commands unavailable in dummy data mode'
        return response


def main(args=None):
    rclpy.init(args=args)
    beluga_pub_serv = BelugaPublisherService(dummy_data_mode=False)
    rclpy.spin(beluga_pub_serv)
    beluga_pub_serv.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
