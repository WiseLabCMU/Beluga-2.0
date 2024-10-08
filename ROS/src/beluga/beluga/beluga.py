import rclpy
from rclpy.node import Node
import typing
from beluga.beluga_serial import BelugaSerial
import json

from beluga_messages.msg import BelugaNeighbor, BelugaNeighbors, BelugaRange, BelugaRanges
from beluga_messages.srv import BelugaATCommand


class BelugaPublisherService(Node):
    DEFAULT_CONFIGS = {
        "boot mode": 2,
        "poll rate": 100,
        "channel": 5,
        "timeout": 9000,
        "tx power": 0,
        "stream mode": 1,
        "twr mode": 1,
        "led mode": 0,
        "format": 1,
        "range extend": 0,
        "uwb data rate": 0,
        "uwb preamble": 1,
        "pulse rate": 1
    }

    def __init__(self):
        super().__init__('beluga')

        # One-off parameters
        self.declare_parameter('neighbors_name', 'neighbors_list')
        pub_topic: str = self.get_parameter('neighbors_name').get_parameter_value().string_value

        self.decalre_parameter('ranges_name', 'range_measurement')
        pub_topic_ranges: str = self.get_parameter('ranges_name').get_parameter_value().string_value

        self.declare_parameter('history_depth', 10)
        pub_history_depth: int = self.get_parameter('history_depth').get_parameter_value().integer_value

        self.declare_parameter('period', 1.0)
        period: float = self.get_parameter('period').get_parameter_value().double_value

        self.declare_parameter('service_topic', 'at_command')
        service_topic: str = self.get_parameter('service_topic').get_parameter_value().string_value

        self.declare_parameter('test_mode', False)
        dummy_data_mode: bool = self.get_parameter('test_mode').get_parameter_value().bool_value

        self.declare_parameter('port', 'port not set')
        port: typing.Optional[str] = self.get_parameter('port').get_parameter_value().string_value
        if port == 'port not set':
            port = None

        self.declare_parameter('config', 'not provided')
        config: typing.Optional[str] = self.get_parameter('config').get_parameter_value().string_value
        if config == 'not provided':
            configs = self.DEFAULT_CONFIGS
        else:
            with open(config, 'r') as f:
                configs = json.load(f)

        self.publisher_ = self.create_publisher(BelugaNeighbors, pub_topic, pub_history_depth)
        self.range_publish_ = self.create_publisher(BelugaRanges, pub_topic_ranges, pub_history_depth)
        self.timer = self.create_timer(period, self.publish_neighbors)
        self.range_timer = self.create_timer(period, self.publish_ranges)
        self.srv = self.create_service(BelugaATCommand, service_topic, self.at_command)
        self.dummy_data = dummy_data_mode
        self.serial: typing.Optional[BelugaSerial] = None
        if not self.dummy_data:
            self.serial: BelugaSerial = BelugaSerial(port=port, logger_func=self.get_logger().info)
            callbacks = {
                "boot mode": self.serial.bootmode,
                "poll rate": self.serial.rate,
                "channel": self.serial.channel,
                "timeout": self.serial.timeout,
                "tx power": self.serial.tx_power,
                "stream mode": self.serial.stream_mode,
                "twr mode": self.serial.twr_mode,
                "led mode": self.serial.led_mode,
                "format": self.serial.format,
                "range extend": self.serial.pwr_amp,
                "uwb data rate": self.serial.datarate,
                "uwb preamble": self.serial.preamble,
                "pulse rate": self.serial.pulserate
            }
            
            # Tell beluga to shut up
            self.serial.stop_ble()
            self.serial.stop_uwb()

            for _config in configs:
                setting = callbacks[_config]()
                self.get_logger().info(f'Current setting: {setting}')
                setting = ''.join([c for c in setting if c.isdigit()])
                if setting:
                    setting = int(setting)
                if setting != configs[_config]:
                    self.get_logger().info(f'Difference in setting. Now setting to {configs[_config]}')
                    response = callbacks[_config](configs[_config])
                    if not response.endswith('OK'):
                        raise RuntimeError(f'Tried setting bad configuration: {setting}, response: {response}')
            self.serial.reboot()
            self.get_logger().info('Ready')
        return

    def publish_neighbors(self):
        neighbors_list = None
        if self.dummy_data:
            neighbors_list = [{'ID': 1, 'RANGE': 0.5761, 'RSSI': -57, 'TIMESTAMP': 5761},
                        {'ID': 2, 'RANGE': 1.7621, 'RSSI': -61, 'TIMESTAMP': 5790},
                        {'ID': 3, 'RANGE': 5.7854, 'RSSI': -72, 'TIMESTAMP': 5004}]
        elif self.serial.neighbors_update:
            neighbors_list = self.serial.neighbors_list

        if neighbors_list is not None:
            pub_list = []
            for x in neighbors_list:
                neighbor = BelugaNeighbor()
                neighbor.id = x['ID']
                neighbor.distance = x['RANGE']
                neighbor.rssi = x['RSSI']
                neighbor.timestamp = x['TIMESTAMP'] # TODO: Sync beluga and ROS
                pub_list.append(neighbor)
            msg = BelugaNeighbors()
            msg.neighbors = pub_list
            self.publisher_.publish(msg)
            self.get_logger().info("Publishing:\n" + '\n'.join(f'{{"ID": {x.id}, "RANGE": {x.distance}, "RSSI": {x.rssi}, "TIMESTAMP": {x.timestamp}}}' for x in pub_list))

    def publish_ranges(self):
        if self.dummy_data:
            return
        elif self.serial.range_update:
            updates = self.serial.range_updates
            range_updates = []
            for x in updates:
                _range = BelugaRange()
                _range.id = x['ID']
                _range.range = x['RANGE']
                range_updates.append(_range)
            msg = BelugaRanges()
            msg.ranges = range_updates
            self.range_publish_.publish(msg)

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
    beluga_pub_serv = BelugaPublisherService()
    rclpy.spin(beluga_pub_serv)
    beluga_pub_serv.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
