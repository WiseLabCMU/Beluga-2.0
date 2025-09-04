/**
 * @file beluga.hpp
 *
 * @brief ROS2 Node for Beluga
 *
 * @date 1/17/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_ROS_BELUGA_HPP
#define BELUGA_ROS_BELUGA_HPP

#include <beluga/beluga_serial.hpp>
#include <beluga_messages/msg/beluga_exchange.hpp>
#include <beluga_messages/msg/beluga_neighbors.hpp>
#include <beluga_messages/msg/beluga_ranges.hpp>
#include <beluga_messages/msg/beluga_unexpected_reboot.hpp>
#include <beluga_messages/srv/beluga_at_command.hpp>
#include <beluga_messages/srv/beluga_power_control.hpp>
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
    [this](auto &&PH1) { _publish_exchange(std::forward<decltype(PH1)>(PH1)); }
#endif // defined(TIMED_PUBLISHERS) || defined(TIMED_RANGE_EVENTS_PUBLISHER)

/**
 * Class to use for the neighbor list. Leave empty to use default class.
 */
#define NEIGHBOR_LIST_CLASS

using namespace std::chrono_literals;

/// ROS2 Node for Beluga
class Beluga : public rclcpp::Node {
    /**
     * Attributes for BelugaSerial
     */
    const BelugaSerial::BelugaSerialAttributes _attr = {
        .neighbor_update_cb = NEIGHBOR_UPDATE_CB,
        .range_updates_cb = RANGE_UPDATE_CB,
        .range_event_cb = RANGE_EVENT_UPDATE_CB,
        .logger_cb =
            [this](auto &&ph_1, auto &&ph_2) {
                return _serial_logger(std::forward<decltype(ph_1)>(ph_1),
                                      std::forward<decltype(ph_2)>(ph_2));
            },
        .unexpected_reboot_event = [this]() { _unexpected_reboot_event(); },
    };

  public:
#pragma clang diagnostic push
#pragma ide diagnostic ignored "modernize-avoid-bind"
    /**
     * Constructor
     */
    Beluga() : Node("beluga"), _serial(_attr) {
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

        this->declare_parameter("reboot_topic", "unexpected_beluga_reboot");

        this->declare_parameter("power_control_topic", "power_control");

        // Timer period for neighbor list publisher (if using the timed
        // publisher)
        this->declare_parameter("neighbor_period", 30);

        // Timer period for range updates publisher (if using the timed
        // publisher)
        this->declare_parameter("ranging_period", 30);

        // Timer period for range events publisher (if using the timed
        // publisher)
        this->declare_parameter("events_period", 100);

        int64_t qos = this->get_parameter("history_depth").as_int();

        _neighbor_list_publisher =
            this->create_publisher<beluga_messages::msg::BelugaNeighbors>(
                this->get_parameter("neighbors_name").as_string(), qos);
        _range_updates_publisher =
            this->create_publisher<beluga_messages::msg::BelugaRanges>(
                this->get_parameter("ranges_name").as_string(), qos);
        _at_command_service =
            this->create_service<beluga_messages::srv::BelugaATCommand>(
                this->get_parameter("service_topic").as_string(),
                std::bind(&Beluga::_run_at_command, this, std::placeholders::_1,
                          std::placeholders::_2));
        _ranging_event_publisher =
            this->create_publisher<beluga_messages::msg::BelugaExchange>(
                this->get_parameter("exchange_name").as_string(), qos);
        _unexpected_reboot = this->create_publisher<
            beluga_messages::msg::BelugaUnexpectedReboot>(
            this->get_parameter("reboot_topic").as_string(), qos);
        _power_control_service =
            this->create_service<beluga_messages::srv::BelugaPowerControl>(
                this->get_parameter("power_control_topic").as_string(),
                std::bind(&Beluga::_update_power_control, this,
                          std::placeholders::_1, std::placeholders::_2));

        _setup();

        _sync_timer = this->create_wall_timer(
            300s, std::bind(&Beluga::_time_sync_helper, this));

#if defined(TIMED_NEIGHBOR_PUBLISHER)
        int64_t neighbor_period =
            this->get_parameter("neighbor_period").as_int();
        _neighbor_timer = this->create_wall_timer(
            std::chrono::seconds(neighbor_period),
            std::bind(&Beluga::_timer_callback_neighbors, this));
#endif // defined(TIMED_NEIGHBOR_PUBLISHER)
#if defined(TIMED_RANGES_PUBLISHER)
        int64_t ranges_period = this->get_parameter("ranging_period").as_int();
        _ranges_timer = this->create_wall_timer(
            std::chrono::seconds(ranges_period),
            std::bind(&Beluga::_timer_callback_ranges, this));
#endif // defined(TIMED_RANGES_PUBLISHER)
#if defined(TIMED_RANGE_EVENTS_PUBLISHER)
        int64_t events_period = this->get_parameter("events_period").as_int();
        _range_events_timer = this->create_wall_timer(
            std::chrono::milliseconds(events_period),
            std::bind(&Beluga::timer_callback_range_events, this));
#endif // defined(TIMED_RANGE_EVENTS_PUBLISHER)
    }
#pragma clang diagnostic pop

  private:
    rclcpp::Publisher<beluga_messages::msg::BelugaNeighbors>::SharedPtr
        _neighbor_list_publisher;
    rclcpp::Publisher<beluga_messages::msg::BelugaRanges>::SharedPtr
        _range_updates_publisher;
    rclcpp::Service<beluga_messages::srv::BelugaATCommand>::SharedPtr
        _at_command_service;
    rclcpp::Publisher<beluga_messages::msg::BelugaExchange>::SharedPtr
        _ranging_event_publisher;
    rclcpp::Publisher<beluga_messages::msg::BelugaUnexpectedReboot>::SharedPtr
        _unexpected_reboot;
    rclcpp::Service<beluga_messages::srv::BelugaPowerControl>::SharedPtr
        _power_control_service;

    rclcpp::TimerBase::SharedPtr _sync_timer;

    void _run_at_command(
        const std::shared_ptr<beluga_messages::srv::BelugaATCommand::Request>
            request,
        std::shared_ptr<beluga_messages::srv::BelugaATCommand::Response>
            response);

    inline std::string pwr_control_cmd(
        uint8_t stage,
        const std::shared_ptr<beluga_messages::srv::BelugaPowerControl::Request>
            request);
    void _update_power_control(
        const std::shared_ptr<beluga_messages::srv::BelugaPowerControl::Request>
            request,
        std::shared_ptr<beluga_messages::srv::BelugaPowerControl::Response>
            response);

    void _publish_neighbor_list(
        const std::vector<BelugaSerial::BelugaNeighbor> &neighbors);
    void
    _publish_ranges(const std::vector<BelugaSerial::BelugaNeighbor> &ranges);
    void _publish_exchange(const BelugaSerial::RangeEvent &event);

    BelugaSerial::BelugaSerial<NEIGHBOR_LIST_CLASS> _serial;

    void _setup();

    static int64_t _extract_number(const std::string &s);
    void _init_time_sync();
    void _resync_time_cb();
    void _time_sync_helper();
    void _time_sync(bool first = false);
    std::tuple<std::string, rclcpp::Time, rclcpp::Time>
    _time_sync_get_measurement();
    double _ns_per_timestamp_unit = 0.0;
    std::map<std::string, std::variant<int64_t, rclcpp::Time>> _last_mapping = {
        {"ros", rclcpp::Time()}, {"beluga", 0}};
    std::mutex _timestamp_sync;
    rclcpp::Time _beluga_to_ros_time(int64_t t);
    static std::map<std::string, int64_t>
    _read_configs(const std::string &config);
    int _serial_logger(const char *msg, va_list args);

#if defined(TIMED_NEIGHBOR_PUBLISHER)
    rclcpp::TimerBase::SharedPtr _neighbor_timer;
    void _timer_callback_neighbors();
#endif // defined(TIMED_NEIGHBOR_PUBLISHER)
#if defined(TIMED_RANGES_PUBLISHER)
    rclcpp::TimerBase::SharedPtr _ranges_timer;
    void _timer_callback_ranges();
#endif // defined(TIMED_RANGES_PUBLISHER)
#if defined(TIMED_RANGE_EVENTS_PUBLISHER)
    rclcpp::TimerBase::SharedPtr _range_events_timer;
    void _timer_callback_range_events();
#endif // defined(TIMED_RANGE_EVENTS_PUBLISHER)

    void _unexpected_reboot_event();
};

#endif // BELUGA_ROS_BELUGA_HPP
