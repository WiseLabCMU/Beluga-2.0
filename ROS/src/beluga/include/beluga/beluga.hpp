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
#include <beluga_messages/msg/beluga_neighbors.hpp>
#include <beluga_messages/msg/beluga_ranges.hpp>
#include <beluga_messages/srv/beluga_at_command.hpp>
#include <beluga_messages/msg/beluga_exchange.hpp>
#include <rclcpp/rclcpp.hpp>

class Beluga : public rclcpp::Node {
  public:
    Beluga() : Node("beluga") {
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
        ranging_event_publisher = this->create_publisher<beluga_messages::msg::BelugaExchange>("ranging_events", 10);
    }

  private:
    rclcpp::Publisher<beluga_messages::msg::BelugaNeighbors>::SharedPtr
        neighbor_list_publisher;
    rclcpp::Publisher<beluga_messages::msg::BelugaRanges>::SharedPtr
        range_updates_publisher;
    rclcpp::Service<beluga_messages::srv::BelugaATCommand>::SharedPtr
        at_command_service;
    rclcpp::Publisher<beluga_messages::msg::BelugaExchange>::SharedPtr ranging_event_publisher;

    void run_at_command(
        const std::shared_ptr<beluga_messages::srv::BelugaATCommand::Request>
            request,
        std::shared_ptr<beluga_messages::srv::BelugaATCommand::Response>
            response);

    void publish_neighbor_list(
        const std::vector<BelugaSerial::BelugaNeighbor> &neighbors);
    void
    publish_ranges(const std::vector<BelugaSerial::BelugaNeighbor> &ranges);
    void publish_exchange(const BelugaSerial::BelugaFrame::RangeEvent &event);

    BelugaSerial::BelugaSerial _serial;
};

#endif // BELUGA_BELUGA_HPP
