/**
 * @file beluga.hpp
 *
 * @brief ROS2 Node for Beluga
 *
 * @date 1/17/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
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

#if defined(TIMED_PUBLISHERS) || defined(TIMED_NEIGHBOR_PUBLISHER)
#ifndef TIMED_NEIGHBOR_PUBLISHER
#define TIMED_NEIGHBOR_PUBLISHER
#endif
#define NEIGHBOR_UPDATE_CB nullptr
#else
#define NEIGHBOR_UPDATE_CB                                                     \
    std::bind(&Beluga::publish_neighbor_list, this, std::placeholders::_1)
#endif // defined(TIMED_PUBLISHERS) || defined(TIMED_NEIGHBOR_PUBLISHER)

#if defined(TIMED_PUBLISHERS) || defined(TIMED_RANGES_PUBLISHER)
#ifndef TIMED_RANGES_PUBLISHER
#define TIMED_RANGES_PUBLISHER
#endif
#define RANGE_UPDATE_CB nullptr
#else
#define RANGE_UPDATE_CB                                                        \
    std::bind(&Beluga::publish_ranges, this, std::placeholders::_1)
#endif // defined(TIMED_PUBLISHERS) || defined(TIMED_RANGES_PUBLISHER)

#if defined(TIMED_PUBLISHERS) || defined(TIMED_RANGE_EVENTS_PUBLISHER)
#ifndef TIMED_RANGE_EVENTS_PUBLISHER
#define TIMED_RANGE_EVENTS_PUBLISHER
#endif
#define RANGE_EVENT_UPDATE_CB nullptr
#else
#define RANGE_EVENT_UPDATE_CB                                                  \
    std::bind(&Beluga::publish_exchange, this, std::placeholders::_1)
#endif // defined(TIMED_PUBLISHERS) || defined(TIMED_RANGE_EVENTS_PUBLISHER)

using namespace std::chrono_literals;

/// ROS2 Node for Beluga
class Beluga : public rclcpp::Node {
    /**
     * Attributes for BelugaSerial
     */
    const BelugaSerial::BelugaSerialAttributes attr = {
        .neighbor_update_cb = NEIGHBOR_UPDATE_CB,
        .range_updates_cb = RANGE_UPDATE_CB,
        .range_event_cb = RANGE_EVENT_UPDATE_CB,
        .logger_cb = std::bind(&Beluga::serial_logger, this,
                               std::placeholders::_1, std::placeholders::_2)};

  public:
    /**
     * Constructor
     */
    Beluga() : Node("beluga"), _serial(attr) {
        // Neighbor list publisher name
        this->declare_parameter("neighbors_name", "neighbor_list");

        // Range updates publisher name
        this->declare_parameter("ranges_name", "range_updates");

        // Range events publisher name
        this->declare_parameter("exchange_name", "range_exchanges");
        this->declare_parameter("history_depth", 10);

        // AT Command service topic name
        this->declare_parameter("service_topic", "at_command");
        this->declare_parameter("port", "");
        this->declare_parameter("config", "");

        // Timer period for neighbor list publisher (if using the timed
        // publisher)
        this->declare_parameter("neighbor_period", 100);

        // Timer period for range updates publisher (if using the timed
        // publisher)
        this->declare_parameter("ranging_period", 100);

        // Timer period for range events publisher (if using the timed
        // publisher)
        this->declare_parameter("events_period", 100);

        int64_t qos = this->get_parameter("history_depth").as_int();

        neighbor_list_publisher =
            this->create_publisher<beluga_messages::msg::BelugaNeighbors>(
                this->get_parameter("neighbors_name").as_string(), qos);
        range_updates_publisher =
            this->create_publisher<beluga_messages::msg::BelugaRanges>(
                this->get_parameter("ranges_name").as_string(), qos);
        at_command_service =
            this->create_service<beluga_messages::srv::BelugaATCommand>(
                this->get_parameter("service_topic").as_string(),
                std::bind(&Beluga::run_at_command, this, std::placeholders::_1,
                          std::placeholders::_2));
        ranging_event_publisher =
            this->create_publisher<beluga_messages::msg::BelugaExchange>(
                this->get_parameter("exchange_name").as_string(), qos);

        _setup();

        sync_timer = this->create_wall_timer(
            300s, std::bind(&Beluga::__time_sync, this));

#if defined(TIMED_NEIGHBOR_PUBLISHER)
        int64_t neighbor_period =
            this->get_parameter("neighbor_period").as_int();
        neighbor_timer = this->create_wall_timer(
            std::chrono::milliseconds(neighbor_period),
            std::bind(&Beluga::timer_callback_neighbors, this));
#endif // defined(TIMED_NEIGHBOR_PUBLISHER)
#if defined(TIMED_RANGES_PUBLISHER)
        int64_t ranges_period = this->get_parameter("ranging_period").as_int();
        ranges_timer = this->create_wall_timer(
            std::chrono::milliseconds(ranges_period),
            std::bind(&Beluga::timer_callback_ranges, this));
#endif // defined(TIMED_RANGES_PUBLISHER)
#if defined(TIMED_RANGE_EVENTS_PUBLISHER)
        int64_t events_period = this->get_parameter("events_period").as_int();
        range_events_timer = this->create_wall_timer(
            std::chrono::milliseconds(events_period),
            std::bind(&Beluga::timer_callback_range_events, this));
#endif // defined(TIMED_RANGE_EVENTS_PUBLISHER)
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

    void _setup();

    static int64_t extract_number(const std::string &s);
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
    static std::map<std::string, int64_t>
    read_configs(const std::string &config);
    int serial_logger(const char *msg, va_list args);

#if defined(TIMED_NEIGHBOR_PUBLISHER)
    rclcpp::TimerBase::SharedPtr neighbor_timer;
    void timer_callback_neighbors();
#endif // defined(TIMED_NEIGHBOR_PUBLISHER)
#if defined(TIMED_RANGES_PUBLISHER)
    rclcpp::TimerBase::SharedPtr ranges_timer;
    void timer_callback_ranges();
#endif // defined(TIMED_RANGES_PUBLISHER)
#if defined(TIMED_RANGE_EVENTS_PUBLISHER)
    rclcpp::TimerBase::SharedPtr range_events_timer;
    void timer_callback_range_events();
#endif // defined(TIMED_RANGE_EVENTS_PUBLISHER)
};

#endif // BELUGA_BELUGA_HPP
