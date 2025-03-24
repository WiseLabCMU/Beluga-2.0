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

static const BelugaSerial::target_pair cmu_beluga = {"CMU", "Beluga"};
static const BelugaSerial::target_pair segger_jlink = {"SEGGER", "J-Link"};

static const std::vector<BelugaSerial::target_pair> TARGETS = {cmu_beluga,
                                                               segger_jlink};

constexpr auto open_delay = 500ms;

static const std::map<BelugaSerial::target_pair, bool> USB_STILL_ALIVE = {
    {cmu_beluga, false}, {segger_jlink, true}};

FileNotFoundError::FileNotFoundError(const char *msg) { _msg = msg; }

const char *FileNotFoundError::what() const noexcept { return _msg.c_str(); }

BelugaSerial::BelugaSerial() { _initialize(); }

BelugaSerial::BelugaSerial(const BelugaSerialAttributes &attr) {
    _initialize(attr);
}

void BelugaSerial::_initialize(const BelugaSerialAttributes &attr) {
    _logger_cb = attr.logger_cb;

    _serial.baudrate(attr.baud);
    _serial.timeout(attr.serial_timeout);
    _serial.exclusive(true);

    if (attr.port.empty()) {
        std::map<BelugaSerial::target_pair, std::vector<std::string>>
            avail_ports;
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
                    std::this_thread::sleep_for(open_delay);
                    auto iterator = USB_STILL_ALIVE.find(target);
                    if (iterator == USB_STILL_ALIVE.end()) {
                        throw std::runtime_error("Unable to determine if port "
                                                 "can stay open during reboot");
                    }
                    _usbRemainsOpen = iterator->second;
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
        _serial.port(attr.port);
        _serial.open();
    }

    _timeout = attr.timeout;

    _neighbor_cb = attr.neighbor_update_cb;
    _range_cb = attr.range_updates_cb;
    _range_event_cb = attr.range_event_cb;
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
    const std::vector<BelugaSerial::target_pair> &valid,
    std::map<BelugaSerial::target_pair, std::vector<std::string>>
        &avail_ports) {
    SerialTools::SysFsScanAttr attr = {
        .ttyXRUSB = false,
        .ttyAMA = false,
        .rfcomm = false,
        .ttyAP = false,
        .ttyGS = false,
    };
    std::vector<SerialTools::SysFS> ports = SerialTools::comports(attr);

    avail_ports.clear();

    for (const auto &port : ports) {
        BelugaSerial::target_pair target = {port.manufacturer(),
                                            port.product()};
        for (const auto &valid_target : valid) {
            if (valid_target == target) {
                avail_ports[target].push_back(port.device());
                break;
            }
        }
    }
}

void BelugaSerial::__process_frames() {
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

void BelugaSerial::_process_frames() {
    while (_tasks_running) {
        try {
            __process_frames();
        } catch (const std::exception &exc) {
            _log("An uncaught exception occurred in processing thread. %s",
                 exc.what());
            std::abort();
        }
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

void BelugaSerial::__read_serial() {
    std::vector<uint8_t> rx;
    std::unique_lock<std::recursive_mutex> lock(_serial_lock, std::defer_lock);

    while (_tasks_running) {
        lock.lock();
        if (_serial.in_waiting() > 0) {
            std::vector<uint8_t> buf;
            _serial.read_all(buf);
            rx.insert(rx.end(), buf.begin(), buf.end());
        }
        lock.unlock();
        _process_rx_buffer(rx);
    }
}

void BelugaSerial::_read_serial() {
    while (_tasks_running) {
        try {
            __read_serial();
        } catch (const Serial::SerialException &) {
            _serial.close();
            // Probably rebooted. Need to attempt reconnection
            std::this_thread::sleep_for(open_delay);
            try {
                _log("Reconnect called from read serial");
                _reconnect();
                if (_time_resync) {
                    std::thread t_(_time_resync);
                    t_.detach();
                }
            } catch (const std::runtime_error &exc) {
                _log(exc.what());
                std::abort();
            }
        } catch (const std::exception &exc) {
            _log("An uncaught exception occurred in reading thread. %s",
                 exc.what());
            std::abort();
        }
    }
}

std::string BelugaSerial::_send_command(const std::string &command) {
    std::vector<uint8_t> tx_data(command.begin(), command.end());
    std::string response;

    try {
        std::lock_guard<std::recursive_mutex> lock(_serial_lock);
        _serial.write(tx_data);
    } catch (Serial::PortNotOpenError &) {
        return "Response timed out";
    }
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
    std::string ret;
    oss << "AT+ID " << id_ << "\r\n";
    ret = _send_command(oss.str());

    if (!id_.empty() && ret.ends_with("OK")) {
        _id = _extract_id(oss.str());
    }
    return ret;
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

std::string BelugaSerial::txpower(const std::string &power) {
    std::stringstream oss;
    oss << "AT+TXPOWER " << power << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::streammode(const std::string &updates_only) {
    std::stringstream oss;
    oss << "AT+STREAMMODE " << updates_only << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::twrmode(const std::string &mode) {
    std::stringstream oss;
    oss << "AT+TWRMODE " << mode << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::ledmode(const std::string &mode) {
    std::stringstream oss;
    oss << "AT+LEDMODE " << mode << "\r\n";
    return _send_command(oss.str());
}

void BelugaSerial::_reboot() {
    const std::string reboot_cmd = "AT+REBOOT\r\n";
    std::lock_guard<std::recursive_mutex> lock(_serial_lock);
    std::vector<uint8_t> tx_data(reboot_cmd.begin(), reboot_cmd.end());
    _serial.write(tx_data);
    _serial.flush();
    _serial.close();
    _log("Called reconnect from reboot");
    _reconnect();
}

std::string BelugaSerial::reboot() {
    std::string ret;
    if (_usbRemainsOpen) {
        _reboot_done.clear();
        ret = _send_command("AT+REBOOT\r\n");
        _reboot_done.wait();
    } else {
        _reboot();
    }
    return ret;
}

std::string BelugaSerial::pwramp(const std::string &mode) {
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

std::string BelugaSerial::phr(const std::string &phr_) {
    std::stringstream oss;
    oss << "AT+PHR " << phr_ << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::pac(const std::string &pac) {
    std::stringstream oss;
    oss << "AT+PAC " << pac << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::sfd(const std::string &sfd) {
    std::stringstream oss;
    oss << "AT+SFD " << sfd << "\r\n";
    return _send_command(oss.str());
}

std::string BelugaSerial::panid(const std::string &pan_id) {
    std::stringstream oss;
    oss << "AT+PANID " << pan_id << "\r\n";
    return _send_command(oss.str());
}

void BelugaSerial::start() {
    if (_tasks_running || !_serial.is_open()) {
        throw std::runtime_error("Please stop before restarting");
    }

    _tasks_running = true;

    _processing_task.task =
        std::packaged_task<void()>([this] { _process_frames(); });
    _processing_task.thread = std::thread(std::move(_processing_task.task));
    _rx_task.task = std::packaged_task<void()>([this] { _read_serial(); });
    _rx_task.thread = std::thread(std::move(_rx_task.task));
    // Ensure that we are in the correct format mode otherwise this program will
    // crash like the Hindenburg
    format("2");
    std::string id_ = id();
    _id = _extract_id(id_);
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

uint16_t BelugaSerial::_extract_id(const std::string &s) {
    std::vector<std::string> tokens;
    size_t start = 0;
    size_t end;

    while ((end = s.find(' ', start)) != std::string::npos) {
        tokens.push_back(s.substr(start, end - start));
        start = end + 1;
    }
    tokens.push_back(s.substr(start));

    for (auto &it : tokens) {
        it.erase(
            std::remove_if(it.begin(), it.end(),
                           [](unsigned char c) { return !std::isprint(c); }),
            it.end());
    }

    std::string id_str;
    for (const auto &it : tokens) {
        bool pure_number = true;
        if (!it.empty()) {
            for (const auto &c : it) {
                if (!std::isdigit(c)) {
                    pure_number = false;
                    break;
                }
            }
        }

        if (pure_number && !it.empty()) {
            id_str = it;
            break;
        }
    }

    if (id_str.empty()) {
        throw std::runtime_error("Unable to parse integer");
    }

    int id_ = std::stoi(id_str);
    if (id_ <= (int)UINT16_MAX && id_ >= 0) {
        return (uint16_t)id_;
    }
    throw std::overflow_error("Unable to convert ID to `uint16_t`");
}

std::vector<std::string>
BelugaSerial::_find_port_candidates(const std::vector<std::string> &skip_list) {
    std::map<BelugaSerial::target_pair, std::vector<std::string>> avail_ports;
    std::vector<std::string> candidates;
    BelugaSerial::_find_ports(TARGETS, avail_ports);

    for (const auto &target : TARGETS) {
        if (avail_ports.find(target) == avail_ports.end()) {
            continue;
        }
        for (const auto &port_ : avail_ports[target]) {
            if (std::find(skip_list.begin(), skip_list.end(), port_) !=
                skip_list.end()) {
                continue;
            }
            candidates.push_back(port_);
        }
    }

    return candidates;
}

bool BelugaSerial::_open_port(std::string &port) {
    bool ret = true;

    std::stringstream oss;
    try {
        oss << "Trying to connect to " << port << "\n";
        _log(oss.str().c_str());
        _serial.port(port);
        _serial.open();
        std::this_thread::sleep_for(open_delay);
    } catch (const Serial::SerialException &exc) {
        _serial.close();
        oss.clear();
        oss << exc.what() << "\n";
        _log(oss.str().c_str());
        ret = false;
    }

    return ret;
}

std::string BelugaSerial::_get_id_from_device() {
    std::vector<uint8_t> at_id = {'A', 'T', '+', 'I', 'D', '\r', '\n'};
    std::vector<uint8_t> response;
    // Wait for initialization
    std::this_thread::sleep_for(500ms);
    try {
        _serial.write(at_id);
    } catch (const Serial::SerialException &exc) {
        if (exc.code() == EIO) {
            return "";
        }
    }
    // Wait for response
    std::this_thread::sleep_for(500ms);
    _serial.read_all(response);

    while (true) {
        auto [frame_start, frame_size, _] = BelugaFrame::frame_present(
            (const char *)response.data(), response.size());
        if (frame_start < 0) {
            _serial.close();
            return "";
        }

        BelugaFrame frame;
        frame.parse_frame(response, frame_start);
        BelugaFrame::DecodedFrame frame_ = frame.get_parsed_data();
        if (frame_.type == BelugaFrame::COMMAND_RESPONSE) {
            std::string payload = std::get<std::string>(frame_.payload);
            if (payload.starts_with("ID:")) {
                return payload;
            }
        }
        response.erase(response.begin() + frame_start,
                       response.begin() + frame_start + frame_size);
    }
}

void BelugaSerial::__reconnect() {
    enum ReconnectStates state = RECONNECT_FIND;
    std::vector<std::string> skips;
    std::vector<std::string> ports;
    std::string port, id_resp;
    std::vector<std::string>::iterator it;

    while (state != RECONNECT_DONE) {
        switch (state) {
        case RECONNECT_FIND: {
            ports = _find_port_candidates(skips);
            it = ports.begin();
            state = (ports.empty()) ? RECONNECT_SLEEP : RECONNECT_CONNECT;
            break;
        }
        case RECONNECT_CONNECT: {
            bool opened = _open_port(*it);
            state = opened ? RECONNECT_GET_ID : RECONNECT_SLEEP;
            break;
        }
        case RECONNECT_GET_ID: {
            id_resp = _get_id_from_device();
            state = id_resp.empty() ? RECONNECT_NEXT : RECONNECT_CHECK_ID;
            break;
        }
        case RECONNECT_CHECK_ID: {
            uint16_t id_ = _extract_id(id_resp);
            state = (id_ == _id) ? RECONNECT_DONE : RECONNECT_UPDATE_SKIPS;
            break;
        }
        case RECONNECT_SLEEP: {
            std::this_thread::sleep_for(open_delay);
            state = RECONNECT_FIND;
            break;
        }
        case RECONNECT_UPDATE_SKIPS: {
            _serial.close();
            skips.push_back(*it);
            state = RECONNECT_NEXT;
            break;
        }
        case RECONNECT_NEXT: {
            it++;
            state = (it == ports.end()) ? RECONNECT_SLEEP : RECONNECT_CONNECT;
            break;
        }
        default:
            _log("reached invalid connection state");
            abort();
            break;
        }
    }
    _log("Connected to %s", it->c_str());
}

void BelugaSerial::_reconnect() {
    std::lock_guard<std::recursive_mutex> lock(_serial_lock);
    std::packaged_task<void()> task(
        std::bind(&BelugaSerial::__reconnect, this));
    auto future = task.get_future();
    std::thread t(std::move(task));
    if (future.wait_for(30s) != std::future_status::timeout) {
        t.join();
        future.get();
    } else {
        throw std::runtime_error("Reconnection timed out");
    }
}
} // namespace BelugaSerial
