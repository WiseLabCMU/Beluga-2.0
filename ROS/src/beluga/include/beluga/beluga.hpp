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


#include <rclcpp/rclcpp.hpp>
#include <beluga_messages/msg/beluga_neighbors.hpp>
#include <beluga_messages/msg/beluga_ranges.hpp>
#include <beluga_messages/srv/beluga_at_command.hpp>

class Beluga : public rclcpp:: Node
{
public:
    Beluga() : Node("beluga") {
        neighbor_list_publisher = this->create_publisher<beluga_messages::msg::BelugaNeighbors>("neighbor_list", 10);
        range_updates_publisher = this->create_publisher<beluga_messages::msg::BelugaRanges>("range_updates", 10);
        at_command_service = this->create_service<beluga_messages::srv::BelugaATCommand>("at_command", std::bind(&Beluga::run_at_command, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    rclcpp::Publisher<beluga_messages::msg::BelugaNeighbors>::SharedPtr neighbor_list_publisher;
    rclcpp::Publisher<beluga_messages::msg::BelugaRanges>::SharedPtr range_updates_publisher;
    rclcpp::Service<beluga_messages::srv::BelugaATCommand>::SharedPtr at_command_service;

    void run_at_command(const std::shared_ptr<beluga_messages::srv::BelugaATCommand::Request> request, std::shared_ptr<beluga_messages::srv::BelugaATCommand::Response> response);
};

#endif //BELUGA_BELUGA_HPP
