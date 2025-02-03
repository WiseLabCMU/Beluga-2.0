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

using beluga_messages::srv::BelugaATCommand;

void Beluga::run_at_command(
    const std::shared_ptr<beluga_messages::srv::BelugaATCommand::Request>
        request,
    std::shared_ptr<beluga_messages::srv::BelugaATCommand::Response> response) {
    switch (request->at_command) {
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

void Beluga::publish_neighbor_list(const std::vector<BelugaSerial::BelugaNeighbor> &neighbors) {
    auto message = beluga_messages::msg::BelugaNeighbors();
    for (const auto &it : neighbors) {
        auto neighbor = beluga_messages::msg::BelugaNeighbor();
        neighbor.id = it.id();
        neighbor.rssi = it.rssi();
        neighbor.distance = (float)it.range();
        neighbor.exchange = it.exchange();
        // TODO: Timestamp
        message.neighbors.push_back(neighbor);
    }
    neighbor_list_publisher->publish(message);
}

void Beluga::publish_ranges(const std::vector<BelugaSerial::BelugaNeighbor> &ranges) {
    auto message = beluga_messages::msg::BelugaRanges();
    for (const auto &it : ranges) {
        auto range = beluga_messages::msg::BelugaRange();
        range.id = it.id();
        range.range = (float)it.range();
        range.exchange = it.exchange();
        // TODO: Timestamp
        message.ranges.push_back(range);
    }
    range_updates_publisher->publish(message);
}

void Beluga::publish_exchange(const BelugaSerial::BelugaFrame::RangeEvent &event) {
    auto message = beluga_messages::msg::BelugaExchange();
    message.id = event.ID;
    message.exchange = event.EXCHANGE;
    // TODO: Timestamp
    ranging_event_publisher->publish(message);
}
