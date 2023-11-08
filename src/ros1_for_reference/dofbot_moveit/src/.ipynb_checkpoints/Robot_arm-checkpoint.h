#ifndef ARM_MOVEIT_ROBTO_ARM_H
#define ARM_MOVEIT_ROBTO_ARM_H

#include <thread>
#include <iostream>
#include <QtWidgets>
#include <ros/ros.h>
#include <QApplication>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;

class Robot_arm : public QWidget {
private:
    // 初始化节点
    ros::NodeHandle n;
    // 初始化客户端
    ros::Subscriber sub;
    // 创建时间更新指针
    QTimer *updateTimer;
    // 创建布局管理器
    QFormLayout *layout;
    // 创建水平布局指针
    QHBoxLayout *layout_pos;
    QHBoxLayout *layout_Target_pos;
    QHBoxLayout *layout_Target_joints;
    //创建按钮控件指针
    QPushButton *btn_random;
    QPushButton *btn_setPos;
    QPushButton *btn_setJoints;
    QPushButton *btn_LevelPos;
    QPushButton *btn_vertPos;
    QPushButton *btn_displayPos;
    QPushButton *btn_displayJoint;
    //设置目标位姿
    QLineEdit *target_X;
    QLineEdit *target_Y;
    QLineEdit *target_Z;
    QLineEdit *target_RZ;
    QLineEdit *target_RY;
    QLineEdit *target_RX;
    //设置目标关节角
    QLineEdit *joint1;
    QLineEdit *joint2;
    QLineEdit *joint3;
    QLineEdit *joint4;
    QLineEdit *joint5;
    //实时显示末端位姿
    QLabel *plan_state;
    QLabel *current_X;
    QLabel *current_Y;
    QLabel *current_Z;
    QLabel *current_RZ;
    QLabel *current_RY;
    QLabel *current_RX;
    //实时显示关节旋转动的角度
    QLabel *current_joint1;
    QLabel *current_joint2;
    QLabel *current_joint3;
    QLabel *current_joint4;
    QLabel *current_joint5;
    // 设置dofbot的状态
    enum motion_status {
        wait,  //等待中
        runing //运行中
    };
    // 弧度转角度
    const float RA2DE = 180.0f / M_PI;
    // 角度转弧度
    const float DE2RA = M_PI / 180.0f;
    int index= 0;
    // 初始机械臂状态为等待
    motion_status status = wait;
public:
    Robot_arm(ros::NodeHandle *n, QWidget *paretn = nullptr);

    ~Robot_arm();

    void thread_setPos();

    void thread_setJoints();

    void thread_vertPos();

    void thread_LevelPos();

    void thread_rdm();

    void thread_displayPos();

    void thread_displayJoint();

    void setPos_call();

    void setJoints_call();

    void vertPos_call();

    void LevelPos_call();

    void rdm_call();

    void pos_display();

    void joint_display();

    void joint_call(const sensor_msgs::JointState &msg);

    void onUpdate();

};

#endif //ARM_MOVEIT_ROBTO_ARM_H
