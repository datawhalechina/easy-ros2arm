#include "dofbot_kinemarics.h"
#include <iostream>
using namespace KDL;
using namespace std;
Dofbot dofbot = Dofbot();
// 弧度转角度
const float RA2DE = 180.0f / M_PI;
// 角度转弧度
const float DE2RA = M_PI / 180.0f;
const char *urdf_file = "/home/jetson/dofbot_ws/src/dofbot_moveit/urdf/dofbot.urdf";

/*
 * 这是求机械臂正解的代码,不涉及夹爪.
 */
int main(int argc, char **argv) {
    // 当前各关节角速度
//    double joints[]{90, 90, 90, 90, 90};
    double joints[]{90, 135, 0, 0, 90};
    // 定义目标关节角容器
    vector<double> initjoints;
    // 定义位姿容器
    vector<double> initpos;
    // 目标关节角度单位转换,由角度[0,180]转换成弧度[-1.57,1.57]
    for (int i = 0; i < 5; ++i) initjoints.push_back((joints[i] - 90) * DE2RA);
    // 调取正解函数,获取当前位姿
    dofbot.dofbot_getFK(urdf_file, initjoints, initpos);
    cout <<fixed<< "FK kinematics result : " << endl;
    cout << "X坐标 (cm)： " << initpos.at(0) * 100 << "\t"
         << "Y坐标 (cm)： " << initpos.at(1) * 100 << "\t"
         << "Z坐标 (cm)： " << initpos.at(2) * 100 << endl;
    cout << "Roll  (°)： " << initpos.at(3) * RA2DE << "\t"
         << "Pitch (°)： " << initpos.at(4) * RA2DE << "\t"
         << "Yaw   (°)： " << initpos.at(5) * RA2DE << endl;
    return 0;
}