#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <std_msgs/msg/color_rgba.hpp>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "attached_object_checking",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    auto const logger = rclcpp::get_logger("dofbot_moveit");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor](){executor.spin();});

    using moveit::planning_interface::MoveGroupInterface;
    
    auto move_group_interface = MoveGroupInterface(node, "dofbot_arm");
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                                                        move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    move_group_interface.startStateMonitor();
    move_group_interface.setStartStateToCurrentState();

    move_group_interface.setMaxAccelerationScalingFactor(1);
    move_group_interface.setMaxVelocityScalingFactor(0.8);
    move_group_interface.setGoalPositionTolerance(0.01);
    move_group_interface.setGoalOrientationTolerance(0.01);
    move_group_interface.setPlanningTime(5);
    move_group_interface.setNumPlanningAttempts(5);

    auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()]{
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "box1";
        
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.1;
        primitive.dimensions[primitive.BOX_Y] = 0.2;
        primitive.dimensions[primitive.BOX_Z] = 0.1;

        geometry_msgs::msg::Pose box_pose;
        box_pose.position.x = 0;
        box_pose.position.y = 0.2;
        box_pose.position.z = 0.15;

        tf2::Quaternion quaternion;
        double Roll = 0.0;
        double Pitch  = 0.0;
        double Yaw = 90.0;
        quaternion.setRPY(Roll * M_PI / 180, Pitch * M_PI / 180, Yaw * M_PI / 180);

        box_pose.orientation.x = quaternion.x();
        box_pose.orientation.y = quaternion.y();
        box_pose.orientation.z = quaternion.z();
        box_pose.orientation.w = quaternion.w();

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std_msgs::msg::ColorRGBA color;
    color.r = 0.2;
    color.g = 0.7;
    color.b = 0.1;
    color.a = 0.5;
    planning_scene_interface.applyCollisionObject(collision_object, color);

    move_group_interface.setNamedTarget("down");
    move_group_interface.move();
    sleep(5);

    move_group_interface.setRandomTarget();

    auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    if(success){
        moveit_visual_tools.trigger();
        move_group_interface.execute(plan);
    }else{
        RCLCPP_ERROR(logger, "Planning faild!");
    }


    rclcpp::shutdown();
    spinner.join();
    return 0;
}
