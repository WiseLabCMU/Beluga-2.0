//
// Created by tom on 10/2/24.
//

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

#include "beluga_messages/srv/beluga_at_command.hpp"
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("update_rate_client");

    rclcpp::Client<beluga_messages::srv::BelugaATCommand>::SharedPtr client =
            node->create_client<beluga_messages::srv::BelugaATCommand>("at_command");

    auto request = std::make_shared<beluga_messages::srv::BelugaATCommand::Request>();
    request->at_command = request->AT_COMMAND_RATE;
    request->arg = "250";

    while(!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", result.get()->response.c_str());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service at_command");
    }

    rclcpp::shutdown();
    return 0;
}

