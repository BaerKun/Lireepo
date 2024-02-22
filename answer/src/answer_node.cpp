#include "rclcpp/rclcpp.hpp"
#include "../include/answer/answer_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto answerNode = std::make_shared<Answer>("answer_node");
    rclcpp::spin(answerNode);
    rclcpp::shutdown();
    return 0;
}
