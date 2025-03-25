import cmd2
from beluga_serial import BelugaSerial, BelugaSerialAttr
from serial import PortNotOpenError
import functools


def serial_command(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except PortNotOpenError:
            print("Port is not open")
            return False
    return wrapper



class BelugaTerminal(cmd2.Cmd):
    intro = "Welcome to The Beluga Terminal Interface. Type help or ? to list commands.\n"
    prompt = "> "
    _serial = BelugaSerial(BelugaSerialAttr(auto_connect=False))
    _neighbors = {}
    _ranges = {}
    _last_exchange = {}

    def _open(self, arg):
        """Opens a targeted port"""
        try:
            self._serial.open_target(arg)
        except FileNotFoundError:
            print("Port not found")
            exit(1)

    def preloop(self) -> None:
        targets = self._serial.find_ports()
        if not targets:
            print("No targets found")
            exit(1)

        options = sum(len(ports) for ports in targets.values())
        if options > 1:
            for target in targets:
                for port in targets[target]:
                    print(f"{target}: {port}")
            port = input("Enter a port name: ")
        else:
            port = list(targets.values())[0][0]
        print(port)
        self._open(port)

    @staticmethod
    def _run_command(cmd, arg):
        if not arg:
            print(cmd())
        else:
            print(cmd(arg))

    @serial_command
    def do_startuwb(self, arg):
        """Start the UWB"""
        print(self._serial.start_uwb())

    @serial_command
    def do_stopuwb(self, arg):
        """Stop the UWB"""
        print(self._serial.stop_uwb())

    @serial_command
    def do_startble(self, arg):
        """Start the BLE"""
        print(self._serial.start_ble())

    @serial_command
    def do_stopble(self, arg):
        """Stop the BLE"""
        print(self._serial.stop_ble())

    @serial_command
    def do_id(self, arg):
        """
        Retrieve the ID of the Beluga node if invoked without an argument.
        Sets the ID of the Beluga node if invoked with an argument.
        """
        self._run_command(self._serial.id, arg)

    @serial_command
    def do_bootmode(self, arg):
        """
        Retrieve the BOOTMODE of the Beluga node if invoked without an argument.
        Sets the BOOTMODE of the Beluga node if invoked with an argument.

        Reference:
            0: Boot with everything off
            1: Boot with BLE on
            2: Boot with BLE and UWB on
        """
        self._run_command(self._serial.bootmode, arg)

    @serial_command
    def do_rate(self, arg):
        """
        Retrieve the RATE of the Beluga node if invoked without an argument.
        Sets the RATE of the Beluga node if invoked with an argument.
        """
        self._run_command(self._serial.rate, arg)

    @serial_command
    def do_channel(self, arg):
        """
        Retrieve the CHANNEL of the Beluga node if invoked without an argument.
        Sets the CHANNEL of the Beluga node if invoked with an argument.
        """
        self._run_command(self._serial.channel, arg)

    @serial_command
    def do_reset(self, arg):
        """Resets the Beluga Node's settings for the next boot"""
        print(self._serial.reset())

    @serial_command
    def do_timeout(self, arg):
        """
        Retrieve the TIMEOUT of the Beluga node if invoked without an argument.
        Sets the TIMEOUT of the Beluga node if invoked with an argument.
        """
        self._run_command(self._serial.timeout, arg)

    @serial_command
    def do_txpower(self, arg):
        """
        Retrieve the TXPOWER of the Beluga node if invoked without an argument.
        Sets the TXPOWER of the Beluga node if invoked with an argument.

        Note that this only affects the transmission power for UWB.

        Reference for 2 arguments:
            0 : Default power
            1 : Maximum power

        Reference for 4 arguments:
            Argument 1 (stage):
                0 : BOOSTNORM
                1 : BOOSTP500
                2 : BOOSTP250
                3 : BOOSTP125

            Argument 2 (coarse gain):
                0 : Off (No output)
                1 : 0 dB Gain
                2 : 2.5 dB Gain
                3 : 5 dB Gain
                ⋮ : 2.5 dB Gain steps
                7 : 15 dB Gain

            Argument 3 (fine gain):
                0 : 0.0 dB Gain
                1 : 0.5 dB Gain
                2 : 1.0 dB Gain
                ⋮ : 0.5 dB Gain Steps
                31: 15.5 dB Gain
        """
        self._run_command(self._serial.txpower, arg)

    @serial_command
    def do_streammode(self, arg):
        """
        Retrieve the STREAMMODE of the Beluga node if invoked without an argument.
        Sets the STREAMMODE of the Beluga node if invoked with an argument.

        Reference:
            0: Displays all neighbors in list regardless of whether they have been updated or not.
            1: Displays only the updated neighbors
        """
        self._run_command(self._serial.streammode, arg)

    @serial_command
    def do_twrmode(self, arg):
        """
        Retrieve the TWRMODE of the Beluga node if invoked without an argument.
        Sets the TWRMODE of the Beluga node if invoked with an argument.

        Reference:
            0: SS+TWR
            1: DS+TWR
        """
        self._run_command(self._serial.twrmode, arg)

    @serial_command
    def do_ledmode(self, arg):
        """
        Retrieve the LEDMODE of the Beluga node if invoked without an argument.
        Sets the LEDMODE of the Beluga node if invoked with an argument.

        Reference:
            0: LEDs on
            1: LEDs off
        """
        self._run_command(self._serial.ledmode, arg)

    @serial_command
    def do_reboot(self, arg):
        """Reboots the Beluga node"""
        print(self._serial.reboot())

    @serial_command
    def do_pwramp(self, arg):
        """
        Retrieve the PWRAMP of the Beluga node if invoked without an argument.
        Sets the PWRAMP of the Beluga node if invoked with an argument.

        Reference:
            0: External power amplifiers off
            1: External power amplifiers on
        """
        self._run_command(self._serial.pwramp, arg)

    @serial_command
    def do_antenna(self, arg):
        """
        Retrieve the ANTENNA of the Beluga node if invoked without an argument.
        Sets the ANTENNA of the Beluga node if invoked with an argument.

        Reference:
            1: Primary BLE Antenna
            2: Secondary BLE Antenna
        """
        self._run_command(self._serial.antenna, arg)

    @serial_command
    def do_time(self, arg):
        """Retrieves the current time reference of the node"""
        print(self._serial.time())

    @serial_command
    def do_deepsleep(self, arg):
        """Places the node into deep sleep and exits the terminal"""
        print(self._serial.deepsleep())
        return True

    @serial_command
    def do_datarate(self, arg):
        """
        Retrieve the DATARATE of the Beluga node if invoked without an argument.
        Sets the DATARATE of the Beluga node if invoked with an argument.

        Reference:
            0: 6.8 Mbps
            1: 850 kbps
            2: 110 kbps
        """
        self._run_command(self._serial.datarate, arg)

    @serial_command
    def do_preamble(self, arg):
        """
        Retrieve the PREAMBLE of the Beluga node if invoked without an argument.
        Sets the PREAMBLE of the Beluga node if invoked with an argument.

        Valid Arguments:
            64
            128
            256
            512
            1024
            1536
            2048
            4096
        """
        self._run_command(self._serial.preamble, arg)

    @serial_command
    def do_pulserate(self, arg):
        """
        Retrieve the PULSERATE of the Beluga node if invoked without an argument.
        Sets the PULSERATE of the Beluga node if invoked with an argument.

        Reference:
            0: 64 MHz
            1: 16 MHz
        """
        self._run_command(self._serial.pulserate, arg)

    def do_neighbors(self, arg):
        """Display the most recent neighbor list."""
        resp = self._serial.get_neighbors()
        if resp[0]:
            self._neighbors = resp[1]
        list_ = self._neighbors
        print("\nID,RSSI,RANGE,TIMESTAMP,EXCHANGE")
        for id_ in list_:
            print(id_, list_[id_]["RSSI"], list_[id_]["RANGE"], list_[id_]["TIMESTAMP"], list_[id_]["EXCHANGE"], sep=',')
        print("")

    def do_ranges(self, arg):
        """Display the recent range reading."""
        resp = self._serial.get_ranges()
        print("\nID,RSSI,RANGE,TIMESTAMP,EXCHANGE")
        for id_ in resp:
            print(id_, resp[id_]["RSSI"], resp[id_]["RANGE"], resp[id_]["TIMESTAMP"], resp[id_]["EXCHANGE"], sep=',')
        print("")

    def do_exchange(self, arg):
        """Display the recent ranging exchange."""
        print(self._serial.get_range_event())

    def do_stream_neighbors(self, arg):
        """Stream neighbor updates. Press CTRL+C to stop streaming."""
        try:
            while True:
                resp = self._serial.get_neighbors()
                if not resp[0]:
                    continue
                list_ = resp[1]
                print("\nID,RSSI,RANGE,TIMESTAMP,EXCHANGE")
                for id_ in list_:
                    print(id_, list_[id_]["RSSI"], list_[id_]["RANGE"], list_[id_]["TIMESTAMP"], list_[id_]["EXCHANGE"], sep=',')
        except KeyboardInterrupt:
            pass
        print("")

    def do_stream_ranges(self, arg):
        """Stream ranging updates. Press CTRL+C to stop streaming."""
        try:
            while True:
                resp = self._serial.get_ranges()
                if not resp:
                    continue
                print("\nID,RSSI,RANGE,TIMESTAMP,EXCHANGE")
                for id_ in resp:
                    print(id_, resp[id_]["RSSI"], resp[id_]["RANGE"], resp[id_]["TIMESTAMP"], resp[id_]["EXCHANGE"], sep=',')
        except KeyboardInterrupt:
            pass
        print("")

    def do_stream_exchanges(self, arg):
        """Stream ranging exchanges. Press CTRL+C to stop streaming."""
        try:
            while True:
                resp = self._serial.get_range_event()
                if not resp:
                    continue
                print(resp)
        except KeyboardInterrupt:
            pass
        print("")

    def do_exit(self, arg):
        """Exit the program"""
        return True

    def postloop(self):
        self._serial.close()


def main():
    # Rename commands
    setattr(BelugaTerminal, 'do_stream-neighbors', BelugaTerminal.do_stream_neighbors)
    setattr(BelugaTerminal, 'do_stream-ranges', BelugaTerminal.do_stream_ranges)
    setattr(BelugaTerminal, 'do_stream-exchanges', BelugaTerminal.do_stream_exchanges)
    del BelugaTerminal.do_stream_neighbors
    del BelugaTerminal.do_stream_ranges
    del BelugaTerminal.do_stream_exchanges
    # Run
    BelugaTerminal().cmdloop()


if __name__ == "__main__":
    main()
