#include "dofbot_kinemarics.h"
#include <iostream>

using namespace KDL;
using namespace std;
// 创建机械臂实例
Dofbot dofbot = Dofbot();
// 弧度转角度
const float RA2DE = 180.0f / M_PI;
// 角度转弧度
const float DE2RA = M_PI / 180.0f;
const char *urdf_file = "/home/jetson/dofbot_ws/src/dofbot_moveit/urdf/dofbot.urdf";

/*
 * 这是求机械臂反解的代码,不涉及夹爪.
 * 注意:反解求出的关节角度只是数据,故可能会出现越界的情况.
 */
int main(int argc, char **argv) {
    // 抓取的位姿
    double Roll = -135;
    double Pitch = 0;
    double Yaw = 0;
    // 求末端位置
    double x = 0;
    double y = 5.5;
    double z = 17.375;
    // 末端位置(单位: m)
    double xyz[]{x, y, z};
    // 末端姿态(单位: 弧度)
    double rpy[]{Roll , Pitch, Yaw };
    // 创建输出角度容器
    vector<double> outjoints;
    // 创建末端位置容器
    vector<double> targetXYZ;
    // 创建末端姿态容器
    vector<double> targetRPY;
    // 末端位置单位转换,由cm转换成m
    for (int k = 0; k < 3; ++k) targetXYZ.push_back(xyz[k] / 100);
    // 末端姿态单位转换,由角度转换成弧度
    for (int l = 0; l < 3; ++l) targetRPY.push_back(rpy[l] * DE2RA);
    // 反解求到达目标点的各关节角度
    dofbot.dofbot_getIK(urdf_file, targetXYZ, targetRPY, outjoints);
    // 打印反解结果
    cout <<fixed<< "IK kinematics result : " << endl;
    for (int i = 0; i < 5; i++) cout << outjoints.at(i) * RA2DE + 90 << "\t";
    cout << endl;
    return 0;
}