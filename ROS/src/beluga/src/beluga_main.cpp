/**
 * @file main.cpp
 *
 * @brief
 *
 * @date 1/17/25
 *
 * @author tom
 */

#include <beluga/beluga.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Beluga>());
    rclcpp::shutdown();
    return 0;
}
