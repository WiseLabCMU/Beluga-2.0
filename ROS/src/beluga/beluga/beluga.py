from decimal import DivisionByZero

import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
import typing
from beluga.beluga_serial import BelugaSerial
import json
from threading import Lock

from beluga_messages.msg import BelugaNeighbor, BelugaNeighbors, BelugaRange, BelugaRanges
from beluga_messages.srv import BelugaATCommand

"""
Conversion factor for converting milliseconds to nanoseconds
"""
MS_CONVERSION_FACTOR = 1000000


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

        self.declare_parameter('ranges_name', 'range_measurement')
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
        self.srv = self.create_service(BelugaATCommand, service_topic, self.at_command)
        self.dummy_data = dummy_data_mode
        self.serial: typing.Optional[BelugaSerial] = None
        self.serial: BelugaSerial = BelugaSerial(port=port, logger_func=self.get_logger().info)
        self.serial.start()
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
        self.get_logger().info('Rebooting beluga')
        response = self.serial.reboot()
        self.get_logger().info(f'Reboot response {response}')
        self.get_logger().info('Done rebooting')

        # Time Sync Stuff
        self._timestamp_sync = Lock()
        self.get_logger().info('Syncing Time')
        self._ns_per_timestamp_unit = 0
        self._last_mapping = {
            'ros': Time(),
            'beluga': 0
        }
        self._time_sync(True)
        time_init = 11
        while time_init > 0:
            # Sync the first few
            self.get_clock().sleep_until(self.get_clock().now() + Duration(nanoseconds=500 * MS_CONVERSION_FACTOR))
            self._time_sync()
            time_init -= 1

        self.timer = self.create_timer(period, self.publish_neighbors)
        # self.range_timer = self.create_timer(period, self.publish_ranges)
        self.sync_timer = self.create_timer(300, self._time_sync)

        self.get_logger().info('Ready')
        return

    def _time_sync_get_measurement(self) -> typing.Tuple[str, Time, Time]:
        req = self.get_clock().now()
        t = self.serial.time()
        resp = self.get_clock().now()
        return t, req, resp

    def _time_sync(self, first: bool = False):
        retries = 5

        while retries > 0:
            # Get measurement 1
            t1_, req1, resp1 = self._time_sync_get_measurement()

            delta = int((resp1 - req1).nanoseconds / 2)
            map1 = req1 + Duration(nanoseconds=delta)

            # Wait 100ms
            self.get_clock().sleep_until(resp1 + Duration(nanoseconds=100 * MS_CONVERSION_FACTOR))

            # Get measurement 2
            t2_, req2, resp2 = self._time_sync_get_measurement()

            delta = int((resp2 - req2).nanoseconds / 2)
            map2 = req2 + Duration(nanoseconds=delta)

            try:
                t1 = int(''.join([c for c in t1_ if c.isdigit()]))
                t2 = int(''.join([c for c in t2_ if c.isdigit()]))
                map_diff = map2 - map1
                t_diff = t2 - t1
                self._timestamp_sync.acquire()
                self._ns_per_timestamp_unit += map_diff.nanoseconds / t_diff
            except ValueError:
                retries -= 1
                if retries > 0:
                    self.get_logger().error(f'Unable to sync time. t1: {t1_}, t2: {t2_}')
                else:
                    self.get_logger().error('Unable to sync time. Will retry in 10 minutes')
                continue
            except ZeroDivisionError:
                retries -= 1
                if retries > 0:
                    self.get_logger().error('No time difference, retrying...')
                    # Assuming all previous data was invalid
                    self._ns_per_timestamp_unit = 0
                    first = True
                else:
                    self.get_logger().error('No time difference. Will retry in 10 minutes')
                self._timestamp_sync.release()
                continue
            else:
                if not first:
                    self._ns_per_timestamp_unit /= 2

                # Calculate average time between the mappings and use that as the new mapping for conversion
                delta = Duration(nanoseconds=(map_diff.nanoseconds / 2))
                self._last_mapping['ros'] = map1 + delta
                self._last_mapping['beluga'] = int(t_diff / 2) + t1
                self.get_logger().info(f'Synced time: {self._last_mapping["beluga"]} -> {self._last_mapping["ros"].seconds_nanoseconds()} ({self._ns_per_timestamp_unit} ns/tick)')
                self._timestamp_sync.release()
                retries = -1

    def _beluga_to_ros_time(self, t: int) -> Time:
        self._timestamp_sync.acquire()
        t_delta = int((t - self._last_mapping['beluga']) * self._ns_per_timestamp_unit)
        ros_time = self._last_mapping['ros']
        self._timestamp_sync.release()
        return ros_time + Duration(nanoseconds=t_delta)

    def publish_neighbors(self):
        update, neighbors_list = self.serial.get_neighbors()
        if update:
            pub_list = []
            for x in neighbors_list.keys():
                neighbor = BelugaNeighbor()
                neighbor.id = x
                neighbor.distance = neighbors_list[x]['RANGE']
                neighbor.rssi = neighbors_list[x]['RSSI']
                timestamp = self._beluga_to_ros_time(neighbors_list[x]['TIMESTAMP'])
                neighbor.timestamp = timestamp.to_msg()
                pub_list.append(neighbor)
            msg = BelugaNeighbors()
            msg.neighbors = pub_list
            self.publisher_.publish(msg)
            self.get_logger().info("Publishing:\n" + '\n'.join(f'{{"ID": {x.id}, "RANGE": {x.distance}, "RSSI": {x.rssi}, "TIMESTAMP": {x.timestamp}}}' for x in pub_list))

    def publish_ranges(self):
        if self.serial.range_update:
            updates = self.serial.range_updates
            range_updates = []
            self._timestamp_sync.acquire()
            for x in updates:
                _range = BelugaRange()
                _range.id = x['ID']
                _range.range = x['RANGE']
                ts = x['TIMESTAMP']
                delta = Duration(nanoseconds=((ts - self._last_mapping['beluga']) * self._ns_per_timestamp_unit))
                ros_ts = self._last_mapping['ros'] + delta
                self.get_logger().info(f'Timestamp: {ros_ts.seconds_nanoseconds()}')
                range_updates.append(_range)
            self._timestamp_sync.release()
            msg = BelugaRanges()
            msg.ranges = range_updates
            self.range_publish_.publish(msg)
            self.get_logger()

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
