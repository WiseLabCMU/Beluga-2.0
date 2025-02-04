/**
 * @file beluga.hpp
 *
 * @brief
 *
 * @date 1/17/25
 *
 * @author tom
 */

#ifndef BELUGA_BELUGA_HPP
#define BELUGA_BELUGA_HPP

#include <beluga/beluga_serial.hpp>
#include <beluga_messages/msg/beluga_exchange.hpp>
#include <beluga_messages/msg/beluga_neighbors.hpp>
#include <beluga_messages/msg/beluga_ranges.hpp>
#include <beluga_messages/srv/beluga_at_command.hpp>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <tuple>

using namespace std::chrono_literals;

class Beluga : public rclcpp::Node {
  public:
    Beluga()
        : Node("beluga"), _serial("", BAUD_115200, 2s, 100ms, 16,
                                  std::bind(&Beluga::publish_neighbor_list,
                                            this, std::placeholders::_1),
                                  std::bind(&Beluga::publish_ranges, this,
                                            std::placeholders::_1),
                                  std::bind(&Beluga::publish_exchange, this,
                                            std::placeholders::_1)) {
        neighbor_list_publisher =
            this->create_publisher<beluga_messages::msg::BelugaNeighbors>(
                "neighbor_list", 10);
        range_updates_publisher =
            this->create_publisher<beluga_messages::msg::BelugaRanges>(
                "range_updates", 10);
        at_command_service =
            this->create_service<beluga_messages::srv::BelugaATCommand>(
                "at_command",
                std::bind(&Beluga::run_at_command, this, std::placeholders::_1,
                          std::placeholders::_2));
        ranging_event_publisher =
            this->create_publisher<beluga_messages::msg::BelugaExchange>(
                "ranging_events", 10);

        sync_timer = this->create_wall_timer(
            300s, std::bind(&Beluga::__time_sync, this));
        resync_timer = this->create_wall_timer(
            1s, std::bind(&Beluga::_resync_time_cb, this));
        resync_timer->cancel();
    }

  private:
    rclcpp::Publisher<beluga_messages::msg::BelugaNeighbors>::SharedPtr
        neighbor_list_publisher;
    rclcpp::Publisher<beluga_messages::msg::BelugaRanges>::SharedPtr
        range_updates_publisher;
    rclcpp::Service<beluga_messages::srv::BelugaATCommand>::SharedPtr
        at_command_service;
    rclcpp::Publisher<beluga_messages::msg::BelugaExchange>::SharedPtr
        ranging_event_publisher;

    rclcpp::TimerBase::SharedPtr sync_timer;
    rclcpp::TimerBase::SharedPtr resync_timer;

    void run_at_command(
        const std::shared_ptr<beluga_messages::srv::BelugaATCommand::Request>
            request,
        std::shared_ptr<beluga_messages::srv::BelugaATCommand::Response>
            response);

    void publish_neighbor_list(
        const std::vector<BelugaSerial::BelugaNeighbor> &neighbors);
    void
    publish_ranges(const std::vector<BelugaSerial::BelugaNeighbor> &ranges);
    void publish_exchange(const BelugaSerial::RangeEvent &event);

    BelugaSerial::BelugaSerial _serial;

    static int64_t extract_time(const std::string &s);
    void _init_time_sync();
    void _resync_time_cb();
    void __time_sync();
    void _time_sync(bool first = false);
    std::tuple<std::string, rclcpp::Time, rclcpp::Time>
    _time_sync_get_measurement();
    double _ns_per_timestamp_unit = 0.0;
    std::map<std::string, std::variant<int64_t, rclcpp::Time>> _last_mapping = {
        {"ros", rclcpp::Time()}, {"beluga", 0}};
    std::mutex _timestamp_sync;
    rclcpp::Time _beluga_to_ros_time(int64_t t);

    void _sigusr1_handler(int sig);
};

#endif // BELUGA_BELUGA_HPP
