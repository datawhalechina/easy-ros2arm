#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Quaternion.h>

using namespace std;
// 角度转弧度
const float DE2RA = M_PI / 180.0f;

int main(int argc, char **argv) {
    //ROS节点初始化
    ros::init(argc, argv, "dofbot_motion_plan_cpp");
    //创建节点句柄
    ros::NodeHandle n;
    // 开启ROS线程
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //初始化机械臂
    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");
    // 允许重新规划
    dofbot.allowReplanning(true);
    // 规划的时间(单位：秒)
    dofbot.setPlanningTime(5);
    // 设置规划尝试次数
    dofbot.setNumPlanningAttempts(10);
    // 设置允许目标姿态误差(单位：米)
    dofbot.setGoalPositionTolerance(0.01);
    // 设置允许目标位置误差(单位：弧度)
    dofbot.setGoalOrientationTolerance(0.01);
    // 设置最大速度
    dofbot.setMaxVelocityScalingFactor(1.0);
    // 设置最大加速度
    dofbot.setMaxAccelerationScalingFactor(1.0);
    dofbot.setNamedTarget("down");
    //开始移动
    dofbot.move();
    sleep(0.5);
    //设置具体位置
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.05957241;
    pose.position.z = 0.1680498;
    // 设置目标姿态
    tf::Quaternion quaternion;
    // RPY的单位是角度值
    double Roll = -140;
    double Pitch = 0.0;
    double Yaw = 0.0;
    // RPY转四元数
    quaternion.setRPY(Roll * DE2RA, Pitch * DE2RA, Yaw * DE2RA);
//    pose.orientation.x = 0.940132;
//    pose.orientation.y = -0.000217502;
//    pose.orientation.z = 0.000375234;
//    pose.orientation.w = -0.340811;
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    string link = dofbot.getEndEffectorLink();
    // 设置目标点
    dofbot.setPoseTarget(pose, link);
    int index = 0;
    // 多次执行,提高成功率
    while (index <= 10) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        // 运动规划
        const moveit::planning_interface::MoveItErrorCode &code = dofbot.plan(plan);
        if (code == code.SUCCESS) {
            ROS_INFO_STREAM("plan success");
            dofbot.execute(plan);
            break;
        } else {
            ROS_INFO_STREAM("plan error");
        }
        index++;
    }
    return 0;
}

