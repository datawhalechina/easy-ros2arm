#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "scenePlanning",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    auto const logger = rclcpp::get_logger("dofbot_mobveit");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor](){executor.spin();});

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

    std::vector<double> targets_joints1 = {0, -1.18, -1.17, 0.79, 0.03};
    std::vector<double> targets_joints2 = {0, -1.21, 0.52, -0.89, 0.08};
    std::vector<double> tool_size = {0.03, 0.03, 0.03};

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0.0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    rclcpp::shutdown();
    spinner.join();

    return 0;
}
