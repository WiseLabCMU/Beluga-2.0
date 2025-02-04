/**
 * @file beluga_serial.cpp
 *
 * @brief
 *
 * @date 1/30/25
 *
 * @author tom
 */

#include <beluga/beluga_serial.hpp>
#include <chrono>
#include <filesystem>
#include <serial/serial.hpp>
#include <serial/tools/list_ports.hpp>
#include <stdexcept>
#include <thread>
#include <utility>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace BelugaSerial {

static const std::vector<target_pair> TARGETS = {{"CMU", "Beluga"},
                                                 {"SEGGER", "J-Link"}};

FileNotFoundError::FileNotFoundError(const char *msg) { _msg = msg; }

const char *FileNotFoundError::what() const noexcept { return _msg.c_str(); }

BelugaSerial::BelugaSerial() { _initialize(); }

BelugaSerial::BelugaSerial(
    const std::string &port, BaudRate baud,
    const std::chrono::milliseconds &timeout,
    const std::chrono::milliseconds &serial_timeout, uint32_t max_lines_read,
    std::function<void(const std::vector<BelugaNeighbor> &)> neighbor_update_cb,
    std::function<void(const std::vector<BelugaNeighbor> &)> range_updates_cb,
    std::function<void(const RangeEvent &)> range_event_cb,
    std::function<int(const char *, va_list)> logger_cb) {
    _initialize(baud, timeout, serial_timeout, max_lines_read, port,
                std::move(neighbor_update_cb), std::move(range_updates_cb),
                std::move(range_event_cb), std::move(logger_cb));
}

void BelugaSerial::_initialize(
    BaudRate baud, const std::chrono::milliseconds &timeout,
    const std::chrono::milliseconds &serial_timeout, uint32_t max_lines_read,
    const std::string &port,
    std::function<void(const std::vector<BelugaNeighbor> &)> neighbor_update_cb,
    std::function<void(const std::vector<BelugaNeighbor> &)> range_updates_cb,
    std::function<void(const RangeEvent &)> range_event_cb,
    std::function<int(const char *, va_list)> logger_cb) {
    _logger_cb = std::move(logger_cb);

    _serial.baudrate(baud);
    _serial.timeout(serial_timeout);
    _serial.exclusive(true);

    if (port.empty()) {
        std::map<target_pair, std::vector<std::string>> avail_ports;
        _find_ports(TARGETS, avail_ports);
        if (avail_ports.empty()) {
            throw FileNotFoundError("Unable to find a given target");
        }

        for (const auto &target : TARGETS) {
            if (avail_ports.find(target) == avail_ports.end()) {
                continue;
            }
            for (const auto &port_ : avail_ports[target]) {
                std::stringstream oss;
                try {
                    oss << "Trying to connect to " << target.first << " "
                        << target.second << ": " << port_ << "\n";
                    _log(oss.str().c_str());
                    _serial.port(port_);
                    _serial.open();
                    break;
                } catch (const Serial::SerialException &exc) {
                    oss.clear();
                    oss << exc.what() << "\n";
                    _log(oss.str().c_str());
                }
            }
        }

        if (!_serial.is_open()) {
            throw FileNotFoundError("Unable to open a target");
        }
    } else {
        _serial.port(port);
        _serial.open();
    }

    _read_max_lines = max_lines_read;
    _timeout = timeout;

    _neighbor_cb = std::move(neighbor_update_cb);
    _range_cb = std::move(range_updates_cb);
    _range_event_cb = std::move(range_event_cb);
}

BelugaSerial::~BelugaSerial() { this->close(); }

void BelugaSerial::_log(const char *msg, ...) {
    if (_logger_cb != nullptr) {
        va_list args;
        va_start(args, msg);
        _logger_cb(msg, args);
        va_end(args);
    }
}

void BelugaSerial::_publish_neighbor_update() {
    if (_neighbors.neighbor_updates()) {
        std::vector<BelugaNeighbor> updates;
        _neighbors.get_neighbors(updates);
        if (_neighbor_cb != nullptr) {
            _neighbor_cb(updates);
        } else {
            _neighbor_queue.put(updates, false);
        }
    }
}

void BelugaSerial::_publish_range_update() {
    if (_neighbors.range_updates()) {
        std::vector<BelugaNeighbor> updates;
        _neighbors.get_updates(updates);
        if (_range_cb != nullptr) {
            _range_cb(updates);
        } else {
            _range_queue.put(updates, false);
        }
    }
}

void BelugaSerial::_publish_range_event(RangeEvent event) {
    if (_range_event_cb != nullptr) {
        _range_event_cb(event);
    } else {
        _range_event_queue.put(event, false);
    }
}

void BelugaSerial::_publish_response(std::string &response) {
    if (_command_sent.is_set()) {
        _command_sent.clear();
        _response_queue.put(response);
    }
}

void BelugaSerial::_process_reboot(const std::string &) {
    _range_queue.clear();
    _neighbor_queue.clear();
    _range_event_queue.clear();
    _neighbors.clear();
    if (_reboot_done.is_set()) {
        _log("Beluga rebooted unexpectedly");
        if (_time_resync != nullptr) {
            std::thread t(_time_resync);
            t.detach();
        }
    } else {
        _reboot_done.set();
    }
}

void BelugaSerial::_find_ports(
    const std::vector<target_pair> &valid,
    std::map<target_pair, std::vector<std::string>> &avail_ports) {
    std::vector<SerialTools::SysFS> ports = SerialTools::comports();

    avail_ports.clear();

    for (const auto &port : ports) {
        target_pair target = {port.manufacturer(), port.product()};
        for (const auto &valid_target : valid) {
            if (valid_target == target) {
                avail_ports[target].push_back(port.device());
                break;
            }
        }
    }
}

void BelugaSerial::_process_frames() {
    while (_tasks_running) {
        BelugaFrame::DecodedFrame frame = _batch_queue.get();

        switch (frame.type) {
        case BelugaFrame::BelugaFrameType::NEIGHBOR_UPDATE:
            _neighbors.update(
                std::get<std::vector<BelugaFrame::NeighborUpdate>>(
                    frame.payload));
            break;
        case BelugaFrame::BelugaFrameType::RANGING_EVENT:
            _publish_range_event(std::get<RangeEvent>(frame.payload));
            break;
        case BelugaFrame::BelugaFrameType::NEIGHBOR_DROP:
            _neighbors.remove(std::get<uint32_t>(frame.payload));
            break;
        case BelugaFrame::BelugaFrameType::COMMAND_RESPONSE:
            _publish_response(std::get<std::string>(frame.payload));
            break;
        case BelugaFrame::BelugaFrameType::START_EVENT:
            _process_reboot(std::get<std::string>(frame.payload));
            break;
        default:
            _log("Invalid frame type");
            break;
        }
        _publish_neighbor_update();
        _publish_range_update();
    }
}

void BelugaSerial::_process_rx_buffer(std::vector<uint8_t> &buf) {
    while (true) {
        auto [frame_start, frame_size, _] =
            BelugaFrame::frame_present((const char *)buf.data(), buf.size());
        if (frame_start < 0) {
            return;
        }

        BelugaFrame frame;
        frame.parse_frame(buf, frame_start);
        BelugaFrame::DecodedFrame frame_ = frame.get_parsed_data();
        _batch_queue.put(frame_, false);
        buf.erase(buf.begin() + frame_start,
                  buf.begin() + frame_start + frame_size);
    }
}

void BelugaSerial::_read_serial() {
    std::vector<uint8_t> rx;

    while (_tasks_running) {
        if (_serial.in_waiting() > 0) {
            std::vector<uint8_t> buf;
            _serial.read_all(buf);
            rx.insert(rx.end(), buf.begin(), buf.end());
        }
        _process_rx_buffer(rx);
    }
}

std::string BelugaSerial::_send_command(const std::string &command) {
    std::vector<uint8_t> tx_data(command.begin(), command.end());
    std::string response;

    _serial.write(tx_data);
    _command_sent.set();
    try {
        response = _response_queue.get(false, _timeout);
    } catch (const BelugaQueueException &exc) {
        if (exc.reason() == BelugaQueueException::QUEUE_EMPTY) {
            response = "Response timed out";
        } else {
            throw;
        }
    }
    return response;
}

std::string BelugaSerial::start_uwb() {
    return _send_command("AT+STARTUWB\r\n");
}

std::string BelugaSerial::stop_uwb() { return _send_command("AT+STOPUWB\r\n"); }

std::string BelugaSerial::start_ble() {
    return _send_command("AT+STARTBLE\r\n");
}

std::string BelugaSerial::stop_ble() { return _send_command("AT+STOPBLE\r\n"); }

std::string BelugaSerial::id(const std::string &id_) {
    std::stringstream oss;
    oss << "AT+ID " << id_ << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::bootmode(const std::string &mode) {
    std::stringstream oss;
    oss << "AT+BOOTMODE " << mode << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::rate(const std::string &rate_) {
    std::stringstream oss;
    oss << "AT+RATE " << rate_ << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::channel(const std::string &channel_) {
    std::stringstream oss;
    oss << "AT+CHANNEL " << channel_ << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::reset() { return _send_command("AT+RESET\r\n"); }

std::string BelugaSerial::timeout(const std::string &timeout_) {
    std::stringstream oss;
    oss << "AT+TIMEOUT " << timeout_ << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::tx_power(const std::string &power) {
    std::stringstream oss;
    oss << "AT+TXPOWER " << power << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::stream_mode(const std::string &updates_only) {
    std::stringstream oss;
    oss << "AT+STREAMMODE " << updates_only << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::twr_mode(const std::string &mode) {
    std::stringstream oss;
    oss << "AT+TWRMODE " << mode << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::led_mode(const std::string &mode) {
    std::stringstream oss;
    oss << "AT+LEDMODE " << mode << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::reboot() {
    _reboot_done.clear();
    std::string ret = _send_command("AT+REBOOT\r\n");
    _reboot_done.wait();
    return ret;
}

std::string BelugaSerial::pwr_amp(const std::string &mode) {
    std::stringstream oss;
    oss << "AT+PWRAMP " << mode << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::antenna(const std::string &antenna) {
    std::stringstream oss;
    oss << "AT+ANTENNA " << antenna << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::time() { return _send_command("AT+TIME\r\n"); }

std::string BelugaSerial::format(const std::string &mode) {
    std::stringstream oss;
    oss << "AT+FORMAT " << mode << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::deepsleep() {
    return _send_command("AT+DEEPSLEEP\r\n");
}

std::string BelugaSerial::datarate(const std::string &rate_) {
    std::stringstream oss;
    oss << "AT+DATARATE " << rate_ << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::preamble(const std::string &preamble) {
    std::stringstream oss;
    oss << "AT+PREAMBLE " << preamble << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::pulserate(const std::string &pr) {
    std::stringstream oss;
    oss << "AT+PULSERATE " << pr << "\r\n";
    return _send_command(oss.str());
}

void BelugaSerial::start() {
    if (_tasks_running) {
        throw std::runtime_error("Please stop before restarting");
    }

    _tasks_running = true;

    _processing_task.task =
        std::packaged_task<void()>([this] { _process_frames(); });
    _processing_task.thread = std::thread(std::move(_processing_task.task));
    _rx_task.task = std::packaged_task<void()>([this] { _read_serial(); });
    _rx_task.thread = std::thread(std::move(_rx_task.task));
}

void BelugaSerial::stop() {
    const int max_attempts = 10;
    if (!_tasks_running) {
        return;
    }
    auto rx_future = _rx_task.task.get_future();
    auto process_future = _processing_task.task.get_future();

    _tasks_running = false;

    while (true) {
        auto status = rx_future.wait_for(10ms);
        if (status == std::future_status::ready) {
            break;
        }
    }
    _rx_task.thread.join();

    int retries = 0;
    for (; retries < max_attempts; retries++) {
        int attempts = 0;
        for (; attempts < max_attempts; attempts++) {
            auto status = process_future.wait_for(10ms);
            if (status == std::future_status::ready) {
                break;
            }
        }

        if (attempts >= max_attempts) {
            // Processing task hung itself. Need to cut it from the tree.
            BelugaFrame::DecodedFrame frame;
            frame.type = BelugaFrame::BelugaFrameType::NO_TYPE;
            _batch_queue.put(frame);
        } else {
            _processing_task.thread.join();
            break;
        }
    }

    if (retries >= max_attempts) {
        throw std::runtime_error("Processing task is still hanging...");
    }
}

void BelugaSerial::close() {
    stop();
    _serial.close();
}

bool BelugaSerial::get_neighbors(std::vector<BelugaNeighbor> &list) {
    bool update = true;
    list.clear();

    try {
        list = _neighbor_queue.get(false);
    } catch (const BelugaQueueException &exc) {
        if (exc.reason() == BelugaQueueException::QUEUE_EMPTY) {
            update = false;
        } else {
            throw;
        }
    }

    return update;
}

void BelugaSerial::get_ranges(std::vector<BelugaNeighbor> &list) {
    list.clear();
    try {
        list = _range_queue.get(false);
    } catch (const BelugaQueueException &exc) {
        if (exc.reason() != BelugaQueueException::QUEUE_EMPTY) {
            throw;
        }
    }
}

RangeEvent BelugaSerial::get_range_event() {
    RangeEvent event{};
    try {
        event = _range_event_queue.get(false);
    } catch (const BelugaQueueException &exc) {
        if (exc.reason() != BelugaQueueException::QUEUE_EMPTY) {
            throw;
        }
    }
    return event;
}

void BelugaSerial::register_resync_cb(std::function<void()> cb) {
    _time_resync = std::move(cb);
}

void BelugaSerial::swap_port(const std::string &port) {
    fs::path path = port;

    if (!fs::exists(path)) {
        throw std::invalid_argument(port + " is not a valid path");
    }

    if (_tasks_running) {
        throw std::runtime_error(
            "Cannot swap ports if port is still in use! PLease call stop() "
            "before calling this function.");
    }

    _serial.close();
    _serial.port(port);
    _serial.open();
}
} // namespace BelugaSerial
