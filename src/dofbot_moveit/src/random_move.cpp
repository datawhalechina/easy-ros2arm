#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "random_move",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    auto const logger = rclcpp::get_logger("dofbot_random_move");
    auto dofbot = moveit::planning_interface::MoveGroupInterface(node, "dofbot_arm");
    dofbot.startStateMonitor();
    while (true)
    {
        dofbot.setRandomTarget();
        auto const [success, plan] = [&dofbot]{
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(dofbot.plan(msg));
            return std::make_pair(ok, msg);
        }();
        if(success){
            dofbot.execute(plan);
        }else{
            RCLCPP_ERROR(logger, "Planning failed!");
        }
        sleep(15);
    }
    
    rclcpp::shutdown();
    return 0;
}