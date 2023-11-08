#include <QApplication>
#include <ros/ros.h>
#include "Robot_arm.h"

using namespace std;

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "Dofbot");
    // 创建节点句柄
    ros::NodeHandle n;
    // 创建窗口实例
    QApplication app(argc, argv);
    // 创建机械臂实例
    Robot_arm dofbot(&n);
    // 显示
    dofbot.show();
    return app.exec();
}

