#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;

int main(int argc, char **argv) {
    //ROS节点初始化
    ros::init(argc, argv, "dofbot_random_move_cpp");
    //创建节点句柄
    ros::NodeHandle n;
    // 设置线程
    ros::AsyncSpinner spinner(1);
    // 开启线程
    spinner.start();
    //初始化机械臂
    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");
    //设置目标点
//    dofbot.setNamedTarget("down");
//    //开始移动
//    dofbot.move();
//    dofbot.asyncMove();
//    sleep(0.1);
    while (true){
        //设置随机目标点
        dofbot.setRandomTarget();
        //开始移动
        dofbot.move();
        sleep(0.5);
    }
    // 阻塞进程
    ros::waitForShutdown();
    return 0;
}
