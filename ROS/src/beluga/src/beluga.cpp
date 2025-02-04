/**
 * @file beluga.cpp
 *
 * @brief
 *
 * @date 1/17/25
 *
 * @author tom
 */

#include <beluga/beluga.hpp>
#include <beluga_messages/srv/beluga_at_command.hpp>
#include <cinttypes>
#include <daw/json/daw_json_link.h>
#include <fstream>

class ValueError : std::exception {
  public:
    ValueError() = default;

    [[nodiscard]] const char *what() const noexcept override {
        return "Invalid value";
    }
};

class ZeroDivisionError : std::exception {
  public:
    ZeroDivisionError() = default;

    [[nodiscard]] const char *what() const noexcept override {
        return "Unable to divide by 0";
    }
};

using beluga_messages::srv::BelugaATCommand;

void Beluga::run_at_command(
    const std::shared_ptr<beluga_messages::srv::BelugaATCommand::Request>
        request,
    std::shared_ptr<beluga_messages::srv::BelugaATCommand::Response> response) {
    switch (request->at_command) {
    case BelugaATCommand::Request::AT_COMMAND_STARTBLE:
        response->response = _serial.start_ble();
        break;
    case BelugaATCommand::Request::AT_COMMAND_STARTUWB:
        response->response = _serial.start_uwb();
        break;
    case BelugaATCommand::Request::AT_COMMAND_STOPBLE:
        response->response = _serial.stop_ble();
        break;
    case BelugaATCommand::Request::AT_COMMAND_STOPUWB:
        response->response = _serial.stop_uwb();
        break;
    case BelugaATCommand::Request::AT_COMMAND_ID:
        response->response = _serial.id(request->arg);
        break;
    case BelugaATCommand::Request::AT_COMMAND_BOOTMODE:
        response->response = _serial.bootmode(request->arg);
        break;
    case BelugaATCommand::Request::AT_COMMAND_RATE:
        response->response = _serial.rate(request->arg);
        break;
    case BelugaATCommand::Request::AT_COMMAND_CHANNEL:
        response->response = _serial.channel(request->arg);
        break;
    case BelugaATCommand::Request::AT_COMMAND_RESET:
        response->response = _serial.reset();
        break;
    case BelugaATCommand::Request::AT_COMMAND_TIMEOUT:
        response->response = _serial.timeout(request->arg);
        break;
    case BelugaATCommand::Request::AT_COMMAND_TXPOWER:
        response->response = _serial.tx_power(request->arg);
        break;
    case BelugaATCommand::Request::AT_COMMAND_STREAMMODE:
        response->response = _serial.stream_mode(request->arg);
        break;
    case BelugaATCommand::Request::AT_COMMAND_TWRMODE:
        response->response = _serial.twr_mode(request->arg);
        break;
    case BelugaATCommand::Request::AT_COMMAND_LEDMODE:
        response->response = _serial.led_mode(request->arg);
        break;
    case BelugaATCommand::Request::AT_COMMAND_REBOOT:
        response->response = _serial.reboot();
        break;
    case BelugaATCommand::Request::AT_COMMAND_PWRAMP:
        response->response = _serial.pwr_amp(request->arg);
        break;
    case BelugaATCommand::Request::AT_COMMAND_ANTENNA:
        response->response = _serial.antenna(request->arg);
        break;
    case BelugaATCommand::Request::AT_COMMAND_TIME:
        response->response = _serial.time();
        break;
    case BelugaATCommand::Request::AT_COMMAND_DEEPSLEEP:
        response->response = _serial.deepsleep();
        break;
    case BelugaATCommand::Request::AT_COMMAND_DATARATE:
        response->response = _serial.datarate(request->arg);
        break;
    case BelugaATCommand::Request::AT_COMMAND_PREAMBLE:
        response->response = _serial.preamble(request->arg);
        break;
    case BelugaATCommand::Request::AT_COMMAND_PULSERATE:
        response->response = _serial.pulserate(request->arg);
        break;
    default:
        response->response = "INVALID";
        RCLCPP_ERROR(this->get_logger(), "Invalid AT Command (%d)",
                     request->at_command);
        break;
    }
}

void Beluga::publish_neighbor_list(
    const std::vector<BelugaSerial::BelugaNeighbor> &neighbors) {
    auto message = beluga_messages::msg::BelugaNeighbors();
    for (const auto &it : neighbors) {
        auto neighbor = beluga_messages::msg::BelugaNeighbor();
        neighbor.id = it.id();
        neighbor.rssi = it.rssi();
        neighbor.distance = (float)it.range();
        neighbor.exchange = it.exchange();
        neighbor.timestamp = _beluga_to_ros_time(it.time());
        message.neighbors.push_back(neighbor);
    }
    neighbor_list_publisher->publish(message);
}

void Beluga::publish_ranges(
    const std::vector<BelugaSerial::BelugaNeighbor> &ranges) {
    auto message = beluga_messages::msg::BelugaRanges();
    for (const auto &it : ranges) {
        auto range = beluga_messages::msg::BelugaRange();
        range.id = it.id();
        range.range = (float)it.range();
        range.exchange = it.exchange();
        range.timestamp = _beluga_to_ros_time(it.time());
        message.ranges.push_back(range);
    }
    range_updates_publisher->publish(message);
}

void Beluga::publish_exchange(const struct BelugaSerial::RangeEvent &event) {
    auto message = beluga_messages::msg::BelugaExchange();
    message.id = event.ID;
    message.exchange = event.EXCHANGE;
    message.timestamp = _beluga_to_ros_time(event.TIMESTAMP);
    ranging_event_publisher->publish(message);
}

void Beluga::_time_sync(bool first) {
    int retries = 5;
    while (retries > 0) {
        auto [t1_, req1, resp1] = _time_sync_get_measurement();
        int64_t delta = (resp1 - req1).nanoseconds() / 2;
        auto map1 = req1 + rclcpp::Duration(std::chrono::nanoseconds(delta));

        // Wait 100ms
        this->get_clock()->sleep_until(resp1 + rclcpp::Duration(100ms));

        auto [t2_, req2, resp2] = _time_sync_get_measurement();
        delta = (resp2 - req2).nanoseconds() / 2;
        auto map2 = req2 + rclcpp::Duration(std::chrono::nanoseconds(delta));

        int64_t t1, t2, t_diff;
        auto map_diff = map2 - map1;
        try {
            t1 = extract_number(t1_);
            t2 = extract_number(t2_);
            t_diff = t2 - t1;
            if (t_diff == 0) {
                throw ZeroDivisionError();
            }

            _timestamp_sync.lock();
            this->_ns_per_timestamp_unit +=
                (double)map_diff.nanoseconds() / (double)t_diff;

        } catch (const ValueError &exc) {
            retries--;
            if (retries > 0) {
                RCLCPP_ERROR(this->get_logger(),
                             "Unable to sync time: t1: %s, t2: %s", t1_.c_str(),
                             t2_.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "Unable to sync time. Will retry in 10 minutes");
            }
            continue;
        } catch (const ZeroDivisionError &exc) {
            retries--;
            if (retries > 0) {
                RCLCPP_ERROR(this->get_logger(),
                             "No time difference, retrying...");
                // Assuming previous data is invalid...
                _ns_per_timestamp_unit = 0;
                first = true;
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "No time difference. Will retry in 10 minutes.");
            }
            continue;
        }
        if (!first) {
            _ns_per_timestamp_unit /= 2.0;
        }

        auto dur_delta = rclcpp::Duration(
            std::chrono::nanoseconds(map_diff.nanoseconds() / 2));
        _last_mapping["ros"] = map1 + dur_delta;
        _last_mapping["beluga"] = (t_diff / 2) + t1;
        RCLCPP_INFO(this->get_logger(), "Synced time: %" PRId64 " -> %lf (%lf)",
                    std::get<int64_t>(_last_mapping["beluga"]),
                    std::get<rclcpp::Time>(_last_mapping["ros"]).seconds(),
                    _ns_per_timestamp_unit);
        _timestamp_sync.unlock();
        retries = -1;
    }
}

std::tuple<std::string, rclcpp::Time, rclcpp::Time>
Beluga::_time_sync_get_measurement() {
    auto req = this->get_clock()->now();
    std::string t = _serial.time();
    auto resp = this->get_clock()->now();

    return {t, req, req};
}

int64_t Beluga::extract_number(const std::string &s) {
    std::string s_num;
    int base = 10;
    int (*check_digit)(int) = isdigit;

    for (const auto &it : s) {

        if (check_digit(it)) {
            s_num += it;
        } else if (!s_num.empty()) {
            if (s_num.size() == 1 && s_num == "0" && (it == 'x' || it == 'X')) {
                base = 16;
                check_digit = isxdigit;
                s_num += it;
                continue;
            }
            break;
        }
    }

    if (s_num.empty()) {
        throw ValueError();
    }

    return std::stoll(s_num, nullptr, base);
}

rclcpp::Time Beluga::_beluga_to_ros_time(int64_t t) {
    _timestamp_sync.lock();
    int64_t delta =
        (int64_t)((double)(t - std::get<int64_t>(_last_mapping["beluga"])) *
                  _ns_per_timestamp_unit);
    auto ros_time = std::get<rclcpp::Time>(_last_mapping["ros"]);
    _timestamp_sync.unlock();
    return ros_time + rclcpp::Duration(std::chrono::nanoseconds(delta));
}

void Beluga::_init_time_sync() {
    const size_t init_time_sync_runs = 11;
    RCLCPP_INFO(this->get_logger(), "Syncing time");
    _ns_per_timestamp_unit = 0.0;
    _last_mapping["ros"] = rclcpp::Time();
    _last_mapping["beluga"] = 0;
    _time_sync(true);

    for (size_t i = 0; i < init_time_sync_runs; i++) {
        this->get_clock()->sleep_until(this->get_clock()->now() +
                                       rclcpp::Duration(500ms));
        _time_sync();
    }
    RCLCPP_INFO(this->get_logger(), "Time is synced");
}

void Beluga::_resync_time_cb() {
    this->sync_timer->cancel();
    _init_time_sync();
    this->sync_timer->reset();
}

void Beluga::__time_sync() { _time_sync(); }

constexpr std::array<std::pair<const char *, int64_t>, 12> DEFAULT_CONFIGS = {{
    {"boot mode", 2},
    {"poll rate", 100},
    {"channel", 5},
    {"timeout", 9000},
    {"tx power", 0},
    {"stream mode", 1},
    {"twr mode", 1},
    {"led mode", 0},
    {"range extend", 0},
    {"uwb data rate", 0},
    {"uwb preamble", 128},
    {"pulse rate", 1},
}};

void Beluga::_setup() {
    std::map<std::string, int64_t> configs;
    for (const auto &[key, value] : DEFAULT_CONFIGS) {
        configs[key] = value;
    }
    if (!this->get_parameter("config").as_string().empty()) {
        // Splice custom configs with default ones
        auto file_configs =
            read_configs(this->get_parameter("config").as_string());
        for (const auto &[key, value] : file_configs) {
            configs[key] = value;
        }
    }

    if (!this->get_parameter("port").as_string().empty()) {
        _serial.swap_port(this->get_parameter("port").as_string());
    }

    _serial.register_resync_cb(std::bind(&Beluga::_resync_time_cb, this));
    _serial.start();

    std::map<std::string, std::function<std::string(const std::string &)>>
        callbacks = {
            {"boot mode", std::bind(&BelugaSerial::BelugaSerial::bootmode,
                                    &this->_serial, std::placeholders::_1)},
            {"poll rate", std::bind(&BelugaSerial::BelugaSerial::rate,
                                    &this->_serial, std::placeholders::_1)},
            {"channel", std::bind(&BelugaSerial::BelugaSerial::channel,
                                  &this->_serial, std::placeholders::_1)},
            {"timeout", std::bind(&BelugaSerial::BelugaSerial::timeout,
                                  &this->_serial, std::placeholders::_1)},
            {"tx power", std::bind(&BelugaSerial::BelugaSerial::stream_mode,
                                   &this->_serial, std::placeholders::_1)},
            {"stream mode", std::bind(&BelugaSerial::BelugaSerial::stream_mode,
                                      &this->_serial, std::placeholders::_1)},
            {"twr mode", std::bind(&BelugaSerial::BelugaSerial::twr_mode,
                                   &this->_serial, std::placeholders::_1)},
            {"led mode", std::bind(&BelugaSerial::BelugaSerial::led_mode,
                                   &this->_serial, std::placeholders::_1)},
            {"range extend", std::bind(&BelugaSerial::BelugaSerial::pwr_amp,
                                       &this->_serial, std::placeholders::_1)},
            {"uwb data rate", std::bind(&BelugaSerial::BelugaSerial::datarate,
                                        &this->_serial, std::placeholders::_1)},
            {"uwb preamble", std::bind(&BelugaSerial::BelugaSerial::preamble,
                                       &this->_serial, std::placeholders::_1)},
            {"pulse rate", std::bind(&BelugaSerial::BelugaSerial::pulserate,
                                     &this->_serial, std::placeholders::_1)},
        };

    // Tel beluga to shut up
    _serial.stop_ble();
    _serial.stop_uwb();

    std::string response;

    for (const auto &it : configs) {
        std::string setting = callbacks[it.first]("");
        RCLCPP_INFO(this->get_logger(), "Current setting: %s", setting.c_str());
        int64_t int_setting;
        try {
            int_setting = extract_number(setting);
        } catch (const ValueError &exc) {
            int_setting = -1;
        }

        if (int_setting != configs[it.first]) {
            RCLCPP_INFO(this->get_logger(),
                        "Difference in setting. Now setting to %" PRId64,
                        configs[it.first]);
            response = callbacks[it.first](std::to_string(configs[it.first]));
            if (!response.ends_with("OK")) {
                std::stringstream oss;
                oss << "Tried setting bad configurations: " << int_setting
                    << ", response: " << response;
                throw std::runtime_error(oss.str());
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "Rebooting beluga");
    response = this->_serial.reboot();
    RCLCPP_INFO(this->get_logger(), "Reboot response: %s", response.c_str());
    RCLCPP_INFO(this->get_logger(), "Done rebooting");

    _init_time_sync();

    RCLCPP_INFO(this->get_logger(), "Ready");
}

std::map<std::string, int64_t> Beluga::read_configs(const std::string &config) {
    std::ifstream file(config);

    if (file.fail()) {
        throw std::runtime_error("Failed to open configuration file");
    }
    std::string json_str((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());
    file.close();

    return daw::json::from_json<std::map<std::string, int64_t>>(json_str);
}

int Beluga::serial_logger(const char *msg, va_list args) {
    va_list args_copy;
    va_copy(args_copy, args);

    int len = vsnprintf(nullptr, 0, msg, args_copy);
    va_end(args_copy);

    if (len < 0) {
        return -1;
    }

    char *buf = new char[len + 1];
    vsnprintf(buf, len + 1, msg, args);

    RCLCPP_INFO(this->get_logger(), "%s", buf);
    delete[] buf;

    return len;
}
