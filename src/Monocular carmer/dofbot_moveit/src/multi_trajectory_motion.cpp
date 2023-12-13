#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/orientation_constraint.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>


bool generate_trajectory(
    moveit::planning_interface::MoveGroupInterface& move_group_interface,
    const std::vector<double>& pose,
    moveit_msgs::msg::RobotTrajectory& trajectory){

    moveit::core::RobotStatePtr start_state(move_group_interface.getCurrentState());
    auto const joint_model_group = start_state->getJointModelGroup(move_group_interface.getName());
    move_group_interface.setJointValueTarget(pose);
    
    auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto const ok = static_cast<bool>(move_group_interface.plan(plan));
        return std::make_pair(ok, plan);
    }();
    if(!success){
        return false;
    }

    start_state->setJointGroupPositions(joint_model_group, pose);
    move_group_interface.setStartState(*start_state);
    trajectory.joint_trajectory.joint_names = plan.trajectory_.joint_trajectory.joint_names;
    for(size_t j = 0; j < plan.trajectory_.joint_trajectory.points.size(); j++){
        trajectory.joint_trajectory.points.push_back(plan.trajectory_.joint_trajectory.points[j]);
    }
    return true;
}


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
    moveit_msgs::msg::RobotTrajectory trajectory;
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                                                        move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    move_group_interface.startStateMonitor();
    move_group_interface.allowReplanning(true);

    move_group_interface.setMaxAccelerationScalingFactor(1);
    move_group_interface.setMaxVelocityScalingFactor(0.8);
    move_group_interface.setPlanningTime(5.0);
    move_group_interface.setNumPlanningAttempts(5);
    move_group_interface.setGoalJointTolerance(0.01);
    move_group_interface.setGoalOrientationTolerance(0.01);

    move_group_interface.setStartStateToCurrentState();

    std::vector<std::vector<double>> poses{
        {1.34, -1.0, -0.61, 0.2, 0},
        {0, 0, 0, 0, 0},
        {-1.16, -0.97, -0.81, -0.79, 3.14}
    };
    for(int i = 0; i < poses.size(); ++i){
        bool flag = generate_trajectory(move_group_interface, poses.at(i), trajectory);
        if(!flag){
            RCLCPP_INFO(logger, "planning faild!");
            return 1;
        }
    }

    moveit::planning_interface::MoveGroupInterface::Plan joinedPlan;
    robot_trajectory::RobotTrajectory rt(move_group_interface.getCurrentState()->getRobotModel(), "dofbot_arm");
    rt.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, 1, 1);
    rt.getRobotTrajectoryMsg(trajectory);
    joinedPlan.trajectory_ = trajectory;

    moveit_visual_tools.publishTrajectoryLine(joinedPlan.trajectory_,
                                            move_group_interface.getRobotModel()->getJointModelGroup("dofbot_arm"));
    moveit_visual_tools.trigger();

    move_group_interface.execute(joinedPlan);
    sleep(5);
    rclcpp::shutdown();
    spinner.join();

    return 0;
}
