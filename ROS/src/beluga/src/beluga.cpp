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

void Beluga::run_at_command(const std::shared_ptr<beluga_messages::srv::BelugaATCommand::Request> request, std::shared_ptr<beluga_messages::srv::BelugaATCommand::Response> response) {
    switch(request->at_command) {
        case BelugaATCommand::Request::AT_COMMAND_ID:
            break;
        default:
            response->response = "INVALID";
            RCLCPP_ERROR(this->get_logger(), "Invalid AT Command (%d)", request->at_command);
            break;
    }
}
