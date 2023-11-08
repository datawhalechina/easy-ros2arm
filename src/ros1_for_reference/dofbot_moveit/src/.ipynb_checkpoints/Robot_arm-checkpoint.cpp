#include "Robot_arm.h"

Robot_arm::Robot_arm(ros::NodeHandle *n, QWidget *parent) : QWidget(parent) {
    // 创建时间更新
    updateTimer = new QTimer(this);
    // 设置时间更新间隔
    updateTimer->setInterval(16);
    // 启动时间更新
    updateTimer->start();
    // 链接时间更新槽函数
    connect(updateTimer, &QTimer::timeout, this, &Robot_arm::onUpdate);
    // 创建桌面对象
    QDesktopWidget *deskdop = QApplication::desktop();
    // 设置每次窗口都弹出在屏幕中心
    move((deskdop->width() - this->width()) / 2, (deskdop->height() - this->height()) / 2);
    // 设置狂口标题
    setWindowTitle("Dofbot控制器");
    // 创建布局管理器
    layout = new QFormLayout;
    /////////////////////  第一层  添加基础控制 /////////////////////
    // 创建水平布局
    layout_pos = new QHBoxLayout;
    // 创建按钮控件
    btn_random = new QPushButton("随机移动");
    btn_vertPos = new QPushButton("垂直位置");
    btn_LevelPos = new QPushButton("水平位置");
    // 将按钮控件添加到布局
    layout_pos->addWidget(btn_random);
    layout_pos->addWidget(btn_vertPos);
    layout_pos->addWidget(btn_LevelPos);
    // 将第一层添加到布局管理器
    layout->addRow(layout_pos);
    /////////////////////  第二层  显示末端位姿 /////////////////////
    // 创建按钮控件
    btn_displayPos = new QPushButton("获取Dofbot位姿");
    // 将第二层添加到布局管理器
    layout->addRow(btn_displayPos);
    /////////////////////  第三层  显示末端角度 /////////////////////
    // 创建按钮控件
    btn_displayJoint = new QPushButton("显示Dofbot关节角");
    // 将第三层添加到布局管理器
    layout->addRow(btn_displayJoint);
    /////////////////////  第四层  设定末端位姿 /////////////////////
    // 创建水平布局
    layout_Target_pos = new QHBoxLayout;
    // 创建按钮控件
    btn_setPos = new QPushButton("设定位姿");
    // 创建单行文本编辑控件
    target_X = new QLineEdit;
    target_Y = new QLineEdit;
    target_Z = new QLineEdit;
    target_RX = new QLineEdit;
    target_RY = new QLineEdit;
    target_RZ = new QLineEdit;
    // 设置占位符
    target_X->setPlaceholderText("X坐标 (cm)");
    target_Y->setPlaceholderText("Y坐标 (cm)");
    target_Z->setPlaceholderText("Z坐标 (cm)");
    target_RX->setPlaceholderText("Roll  (°)");
    target_RY->setPlaceholderText("Pitch (°)");
    target_RZ->setPlaceholderText("Yaw   (°)");
    // 添加文本控件到布局
    layout_Target_pos->addWidget(target_X);
    layout_Target_pos->addWidget(target_Y);
    layout_Target_pos->addWidget(target_Z);
    layout_Target_pos->addWidget(target_RX);
    layout_Target_pos->addWidget(target_RY);
    layout_Target_pos->addWidget(target_RZ);
    // 添加按钮控件到布局
    layout_Target_pos->addWidget(btn_setPos);
    // 将第四层添加到布局管理器
    layout->addRow(layout_Target_pos);
    /////////////////////  第五层  设定关节角度 /////////////////////
    // 创建水平布局
    layout_Target_joints = new QHBoxLayout;
    // 创建按钮控件
    btn_setJoints = new QPushButton("设定角度");
    // 创建单行文本编辑控件
    joint1 = new QLineEdit;
    joint2 = new QLineEdit;
    joint3 = new QLineEdit;
    joint4 = new QLineEdit;
    joint5 = new QLineEdit;
    // 设置占位符
    joint1->setPlaceholderText("关节1 [0~180]: ");
    joint2->setPlaceholderText("关节2 [0~180]: ");
    joint3->setPlaceholderText("关节3 [0~180]: ");
    joint4->setPlaceholderText("关节4 [0~180]: ");
    joint5->setPlaceholderText("关节5 [0~270]: ");
    // 添加文本控件到布局
    layout_Target_joints->addWidget(joint1);
    layout_Target_joints->addWidget(joint2);
    layout_Target_joints->addWidget(joint3);
    layout_Target_joints->addWidget(joint4);
    layout_Target_joints->addWidget(joint5);
    // 添加按钮控件到布局
    layout_Target_joints->addWidget(btn_setJoints);
    // 将第五层添加到布局管理器
    layout->addRow(layout_Target_joints);
    /////////////////////  第六层 实时显示末端位姿  /////////////////////
    // 创建标签控件
    plan_state = new QLabel("");
    current_X = new QLabel("");
    current_Y = new QLabel("");
    current_Z = new QLabel("");
    current_RX = new QLabel("");
    current_RY = new QLabel("");
    current_RZ = new QLabel("");
    // 将第六层添加到布局管理器
    layout->addRow("路径规划 : ", plan_state);
    layout->addRow("当前坐标X (cm): ", current_X);
    layout->addRow("当前坐标Y (cm): ", current_Y);
    layout->addRow("当前坐标Z (cm): ", current_Z);
    layout->addRow("当前姿态Roll  (°): ", current_RX);
    layout->addRow("当前姿态Pitch (°): ", current_RY);
    layout->addRow("当前姿态Yaw   (°): ", current_RZ);
    /////////////////////  第七层  实时显示关节角度 /////////////////////
    // 创建标签控件
    current_joint1 = new QLabel;
    current_joint2 = new QLabel;
    current_joint3 = new QLabel;
    current_joint4 = new QLabel;
    current_joint5 = new QLabel;
    // 将第七层添加到布局管理器
    layout->addRow("关节1 [0~180]: ", current_joint1);
    layout->addRow("关节2 [0~180]: ", current_joint2);
    layout->addRow("关节3 [0~180]: ", current_joint3);
    layout->addRow("关节4 [0~180]: ", current_joint4);
    layout->addRow("关节5 [0~270]: ", current_joint5);
    // 设置当前页面布局
    setLayout(layout);
    // 设置末端位姿链接
    connect(btn_setPos, &QPushButton::clicked, this, &Robot_arm::thread_setPos);
    // 设置关节角度链接
    connect(btn_setJoints, &QPushButton::clicked, this, &Robot_arm::thread_setJoints);
    // 设置随机移动链接
    connect(btn_random, &QPushButton::clicked, this, &Robot_arm::thread_rdm);
    // 设置垂直位置链接
    connect(btn_vertPos, &QPushButton::clicked, this, &Robot_arm::thread_vertPos);
    // 设置水平位置链接
    connect(btn_LevelPos, &QPushButton::clicked, this, &Robot_arm::thread_LevelPos);
    // 设置显示位姿链接
    connect(btn_displayPos, &QPushButton::clicked, this, &Robot_arm::thread_displayPos);
    // 设置显示角度链接
    connect(btn_displayJoint, &QPushButton::clicked, this, &Robot_arm::thread_displayJoint);

};

Robot_arm::~Robot_arm() {

};

//////////////////////////////////  线程函数与按钮链接   //////////////////////////////////
// 设置位姿移动线程函数
void Robot_arm::thread_setPos() {
    // 开启线程移动
    std::thread t_setPos(&Robot_arm::setPos_call, this);
    // 线程脱离
    t_setPos.detach();
};

// 设置角度移动线程函数
void Robot_arm::thread_setJoints() {
    // 开启线程移动
    std::thread t_setJoints(&Robot_arm::setJoints_call, this);
    t_setJoints.detach();
};

// 移动水平位置线程函数
void Robot_arm::thread_LevelPos() {
    // 开启线程移动
    std::thread t_LevelPos(&Robot_arm::LevelPos_call, this);
    t_LevelPos.detach();
};

// 移动垂直位置线程函数
void Robot_arm::thread_vertPos() {        // 开启线程移动
    std::thread t_vertPos(&Robot_arm::vertPos_call, this);
    t_vertPos.detach();
};

// 移动随机位置线程函数
void Robot_arm::thread_rdm() {
    std::thread t_rdm(&Robot_arm::rdm_call, this);
    t_rdm.detach();
};

// 显示位姿线程函数
void Robot_arm::thread_displayPos() {
    // 开启位姿信息更新
    index = 1;
    // 开启线程调用
    std::thread t_displayPos(&Robot_arm::pos_display, this);
    t_displayPos.detach();
};

// 显示角度线程函数
void Robot_arm::thread_displayJoint() {
    // 开启线程调用
    std::thread t__displayJoint(&Robot_arm::joint_display, this);
    t__displayJoint.detach();
};

//////////////////////////////////  线程回调回调函数   //////////////////////////////////
// 设置位姿回调函数
void Robot_arm::setPos_call() {
    // 初始化机械臂
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
    geometry_msgs::Pose pose;
    // 获取文本框的位置数值(单位由cm转换成m)
    pose.position.x = target_X->text().toDouble() / 100;
    pose.position.y = target_Y->text().toDouble() / 100;
    pose.position.z = target_Z->text().toDouble() / 100;
    tf::Quaternion quaternion;
    double RX, RY, RZ;
    // 获取文本框的姿态数值(单位由角度转换成弧度)
    RX = (target_RX->text().toDouble()) * DE2RA;
    RY = (target_RY->text().toDouble()) * DE2RA;
    RZ = (target_RZ->text().toDouble()) * DE2RA;
    // 将RPY转换成四元数
    quaternion.setRPY(RX, RY, RZ);
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    // 设置目标点
    dofbot.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    // 路径规划
    const moveit::planning_interface::MoveItErrorCode &code = dofbot.plan(plan);
    if (code == code.SUCCESS) {
        ROS_INFO_STREAM("plan success");
        // 将路径规划成功的信息显示在窗口上
        plan_state->setText(QString::fromStdString("plan success"));
        // 执行运动
        dofbot.execute(plan);
    } else {
        ROS_INFO_STREAM("plan error");
        // 将路径规划失败的信息显示在窗口上
        plan_state->setText(QString::fromStdString("plan error"));
    }
};

// 设置角度回调函数
void Robot_arm::setJoints_call() {
    // 初始化机械臂
    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");
    // 允许重新规划
    dofbot.allowReplanning(true);
    // 规划的时间(单位：秒)
    dofbot.setPlanningTime(5);
    // 设置规划尝试次数
    dofbot.setNumPlanningAttempts(10);
    // 设置允许目标角度误差(单位：弧度)
    dofbot.setGoalJointTolerance(0.01);
    // 设置最大速度
    dofbot.setMaxVelocityScalingFactor(1.0);
    // 设置最大加速度
    dofbot.setMaxAccelerationScalingFactor(1.0);
    vector<double> joints;
    // 获取文本框里各关节角度(单位由角度[0~180]转化为弧度[±1.57])
    joints.push_back((joint1->text().toDouble() - 90) * DE2RA);
    joints.push_back((joint2->text().toDouble() - 90) * DE2RA);
    joints.push_back((joint3->text().toDouble() - 90) * DE2RA);
    joints.push_back((joint4->text().toDouble() - 90) * DE2RA);
    joints.push_back((joint5->text().toDouble() - 90) * DE2RA);
//    ROS_INFO_STREAM((joint1->text().toDouble() - 90) * DE2RA);
//    ROS_INFO_STREAM((joint2->text().toDouble() - 90) * DE2RA);
//    ROS_INFO_STREAM((joint3->text().toDouble() - 90) * DE2RA);
//    ROS_INFO_STREAM((joint4->text().toDouble() - 90) * DE2RA);
//    ROS_INFO_STREAM((joint5->text().toDouble() - 90) * DE2RA);
    // 设置目标点
    bool b = dofbot.setJointValueTarget(joints);
    if (b) {
        ROS_INFO_STREAM("set joints success");
        // 执行运动
        dofbot.move();
    } else {
        ROS_INFO_STREAM("set joints error");
    }
};

// 设置水平位置回调函数
void Robot_arm::vertPos_call() {
    // 初始化机械臂
    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");
    // 设置水平目标
    dofbot.setNamedTarget("up");
    // 执行运动
    dofbot.move();
};

// 设置垂直位置回调函数
void Robot_arm::LevelPos_call() {
    // 初始化机械臂
    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");
    // 设置垂直目标
    dofbot.setNamedTarget("down");
    // 执行运动
    dofbot.move();
};

// 设置随机位置回调函数
void Robot_arm::rdm_call() {
    // 初始化机械臂
    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");
    // 设置随机目标
    dofbot.setRandomTarget();
    // 执行运动
    dofbot.move();
};

// 显示当前位姿信息
void Robot_arm::pos_display() {
    //初始化机械臂
    moveit::planning_interface::MoveGroupInterface arm("dofbot");
    //获取终端link的名称
    std::string end_effector_link = arm.getEndEffectorLink();
    // 获取当前位姿数据最为机械臂运动的起始位姿
    geometry_msgs::Pose start_pose = arm.getCurrentPose(end_effector_link).pose;
    double x = start_pose.position.x;
    double y = start_pose.position.y;
    double z = start_pose.position.z;
//    cout<<start_pose.position.x<<"\t"
//        <<start_pose.position.y<<"\t"
//        <<start_pose.position.z<<"\t"
//        <<start_pose.orientation.x<<"\t"
//        <<start_pose.orientation.y<<"\t"
//        <<start_pose.orientation.z<<"\t"
//        <<start_pose.orientation.w<<endl;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(start_pose.orientation, quat);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    current_X->setText(QString::fromStdString(to_string(x * 100)));
    current_Y->setText(QString::fromStdString(to_string(y * 100)));
    current_Z->setText(QString::fromStdString(to_string(z * 100)));
    current_RX->setText(QString::fromStdString(to_string(roll * 180 / M_PI)));
    current_RY->setText(QString::fromStdString(to_string(pitch * 180 / M_PI)));
    current_RZ->setText(QString::fromStdString(to_string(yaw * 180 / M_PI)));
};

// 显示当前关节角度信息
void Robot_arm::joint_display() {
    // 创建"/joint_states"话题接收者
    sub = n.subscribe("/joint_states", 1, &Robot_arm::joint_call, this);
};

// 获取当前关节角度的回调函数
void Robot_arm::joint_call(const sensor_msgs::JointState &msg) {
    // 将关节角度显示在窗口上(单位由弧度转换为角度[0~180])
    current_joint1->setText(QString::fromStdString(to_string(msg.position[0] * RA2DE + 90)));
    current_joint2->setText(QString::fromStdString(to_string(msg.position[1] * RA2DE + 90)));
    current_joint3->setText(QString::fromStdString(to_string(msg.position[2] * RA2DE + 90)));
    current_joint4->setText(QString::fromStdString(to_string(msg.position[3] * RA2DE + 90)));
    current_joint5->setText(QString::fromStdString(to_string(msg.position[4] * RA2DE + 90)));
};

// 窗口更新函数
void Robot_arm::onUpdate() {
    ros::spinOnce();
    update();
//    Robot_arm::thread_displayPos();
//    if(index!=0){
//        if (index%10==0) Robot_arm::thread_displayPos();
//        index++;
//    }
    if (!ros::ok()) {
        close();
    };
}
