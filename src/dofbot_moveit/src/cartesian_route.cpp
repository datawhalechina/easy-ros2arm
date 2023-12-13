#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "cartesian_route",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    auto const logger = rclcpp::get_logger("dofbot_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor](){executor.spin();});
    
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "dofbot_arm");

    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        node,
        "base_link",
        rviz_visual_tools::RVIZ_MARKER_TOPIC,
        move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();
    
    move_group_interface.allowReplanning(true);

    move_group_interface.startStateMonitor();
    move_group_interface.setStartStateToCurrentState();

    move_group_interface.setMaxAccelerationScalingFactor(1);
    move_group_interface.setMaxVelocityScalingFactor(0.8);
    move_group_interface.setGoalPositionTolerance(0.01);
    move_group_interface.setGoalOrientationTolerance(0.01);
    move_group_interface.setPlanningTime(50);
    move_group_interface.setNumPlanningAttempts(10);

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0037618483876896;
    pose.position.y = 0.1128923321179022;
    pose.position.z = 0.3998656334826569;
    pose.orientation.x = -0.0042810851906468;
    pose.orientation.y = -0.0033330592972940;
    pose.orientation.z = 0.6827314913817025;
    pose.orientation.w = 0.7306492138509612;

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
            break;
        }else{
            RCLCPP_ERROR(logger, "Planning faild!");
        }
        index ++;
    }

    geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose(move_group_interface.getEndEffector()).pose;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start_pose);

    geometry_msgs::msg::Pose target_pose = start_pose;
    target_pose.position.z -= 0.01;
    waypoints.push_back(target_pose);
    target_pose.position.y -= 0.01;
    waypoints.push_back(target_pose);
    target_pose.position.z -= 0.01;
    waypoints.push_back(target_pose);
    target_pose.position.y += 0.01;
    waypoints.push_back(target_pose);
    target_pose.position.z -= 0.01;
    waypoints.push_back(target_pose);
    target_pose.position.y -= 0.01;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = 0.0;

    sleep(5.0);

    int max_attempts_num = 1000;
    int attempts = 0;
    while(fraction < 1.0 && attempts < max_attempts_num){
        fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        RCLCPP_INFO(logger, "Visualizing plan 6 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
        attempts ++;
    }
    if(fraction == 1.0){
        RCLCPP_INFO(logger, "executing trajectory!");
        move_group_interface.execute(trajectory);
    }else{
        RCLCPP_ERROR(logger, "planning error!");
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}