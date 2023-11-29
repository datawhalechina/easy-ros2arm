#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "randomMove",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    auto const logger = rclcpp::get_logger("dofbot_moveit");

    using moveit::planning_interface::MoveGroupInterface;
    
    auto move_group_interface = MoveGroupInterface(node, "dofbot_arm");

    move_group_interface.setMaxAccelerationScalingFactor(0.5);
    move_group_interface.setMaxVelocityScalingFactor(0.5);
    move_group_interface.setPlanningTime(5.0);
    move_group_interface.setNumPlanningAttempts(5);
    move_group_interface.setGoalJointTolerance(0.01);
    move_group_interface.setGoalOrientationTolerance(0.01);

    move_group_interface.setStartState(*move_group_interface.getCurrentState(10));

    move_group_interface.setRandomTarget();

    auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if(success){
        move_group_interface.execute(plan);
    }else{
        RCLCPP_ERROR(logger, "Planning faild!");
    }

    rclcpp::shutdown();
    return 0;
}
