#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std::chrono_literals;

void execute_dofbot_plan(moveit::planning_interface::MoveGroupInterface& dofbot, const rclcpp::Logger& logger){
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto ok = static_cast<bool>(dofbot.plan(msg));
    if(ok){
        dofbot.execute(msg);
    }else{
        RCLCPP_INFO(logger, "Planning failed!");
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto const logger = rclcpp::get_logger("random_move");
    RCLCPP_INFO(logger, "Setting target......");
    auto const node = std::make_shared<rclcpp::Node>("random_move");
    auto dofbot = moveit::planning_interface::MoveGroupInterface(node, "dofbot");
    dofbot.setNamedTarget("down");
    
    RCLCPP_INFO(logger, "Planning......");
    execute_dofbot_plan(dofbot, logger);
    sleep(5);

    RCLCPP_INFO(logger, "Random target Planning......");
    dofbot.setRandomTarget();
    execute_dofbot_plan(dofbot, logger);
    rclcpp::shutdown();
    return 0;
}
