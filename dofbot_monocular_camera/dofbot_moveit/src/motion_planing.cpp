#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.h>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "motionPlanning",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    auto const logger = rclcpp::get_logger("dofbot_moveit");
    using moveit::planning_interface::MoveGroupInterface;

    auto move_group_interface = MoveGroupInterface(node, "dofbot_arm");

    move_group_interface.startStateMonitor();

    move_group_interface.setMaxAccelerationScalingFactor(1);
    move_group_interface.setMaxVelocityScalingFactor(0.8);
    move_group_interface.setPlanningTime(5.0);
    move_group_interface.setNumPlanningAttempts(5);
    move_group_interface.setGoalJointTolerance(0.01);
    move_group_interface.setGoalOrientationTolerance(0.01);

    move_group_interface.setStartStateToCurrentState();

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0037618483876896;
    pose.position.y = 0.1128923321179022;
    pose.position.z =  0.3998656334826569;

    pose.orientation.x = -0.0042810851906468;
    pose.orientation.y = -0.0033330592972940;
    pose.orientation.z = 0.6827314913817025;
    pose.orientation.w = 0.7306492138509612;

    std::string endLink = move_group_interface.getEndEffectorLink();
    move_group_interface.setPoseTarget(pose);

    int index = 0;
    while(index <= 10){
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
        index ++;
    }
    rclcpp::shutdown();
    return 0;
}
