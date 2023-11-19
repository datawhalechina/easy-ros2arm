#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "dofbot_random_move",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    auto const logger = rclcpp::get_logger("dofbot_random_move");
    auto dofbot = moveit::planning_interface::MoveGroupInterface(node, "dofbot_arm");
    while (true)
    {
        dofbot.setRandomTarget();
        dofbot.move();
        sleep(120);
    }
    
    rclcpp::shutdown();
    return 0;
}