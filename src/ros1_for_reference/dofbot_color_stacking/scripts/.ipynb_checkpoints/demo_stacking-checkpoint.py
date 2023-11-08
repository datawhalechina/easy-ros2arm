#!/usr/bin/env python
# coding: utf-8
import random
import threading
from dofbot_config import *
from stacking_target import stacking_GetTarget

# 创建获取目标实例
target = stacking_GetTarget()
# 创建相机标定实例
calibration = Arm_Calibration()
# 创建滑动条实例
calibration_trackbar = calibration_Trackbar()
# 初始化标定方框边点
dp = []
# 初始化抓取信息
msg = {}
init = 0
# 初始化模式
model = None
# 初始化1,2舵机角度值
xy = [90, 135]
# 初始化二值图阈值
thresh = 130
# 初始化颜色选择列表
color_list = {}
# HSV参数路径
HSV_path = "HSV_config.txt"
# XYT参数路径
XYT_path = "XYT_config.txt"
# 初始化文字
txt1 = 'Button-control: 1 ~ Calibration,'
txt2 = '2 ~ CalOK, 3 ~ Detection, 4 ~ ResetColorList'
txt3 = 'q ~ CalCancel, Space ~ Grap, esc ~ Quit'
# 初始化HSV值
color_hsv = {"red": ((2, 100, 60), (11, 255, 200)),
             "green": ((55, 80, 20), (78, 255, 86)),
             "blue": ((110, 100, 30), (125, 255, 200)),
             "yellow": ((26, 100, 100), (32, 232, 230))}


def get_color_name():
    global color_list
    global init
    if init == 0:
        print("--------------------------------------------")
        print("选择识别的颜色序列: 1 , 23 , 234 , 1234")
        print("['1'：红色 '2'：绿色 '3'：蓝色 '4'：黄色]")
        init = 1
    num = str(input("请输入选择识别的颜色序列:"))
    if 0 < len(num) <= 4:
        for index, value in enumerate(num):
            if value == "1": color_list[str(index + 1)] = "red"
            if value == "2": color_list[str(index + 1)] = "green"
            if value == "3": color_list[str(index + 1)] = "blue"
            if value == "4": color_list[str(index + 1)] = "yellow"
    else:
        print("输入不规范,请重新输入!")
        color_list = {}


if __name__ == '__main__':
    # 读取更新后的HSV参数
    try: read_HSV(HSV_path, color_hsv)
    except Exception: print("No HSV_config file!!!")
    # 读取更新后的XYT参数
    try: xy, thresh = read_XYT(XYT_path)
    except Exception: print("No XYT_config file!!!")
    # 打开摄像头
    capture = cv.VideoCapture(0)
    # 创建滑动条
    calibration_trackbar.create_bar("img", xy, thresh)
    # 当摄像头正常打开的情况下循环执行
    while capture.isOpened():
        # 读取相机的每一帧
        _, img = capture.read()
        # 统一图像大小
        img = cv.resize(img, (640, 480))
        # 获取滑动条的值
        (xy, thresh) = calibration_trackbar.get_result()
        # 水平翻转
        # img = cv.flip(img, -1)
        action = cv.waitKey(1) & 0xff
        # 标定模式 按键:"1"
        if action == ord('1'): model = 'Calibration'
        # 标定确认 按键:"2"
        if action == ord('2') and model == 'Calibration':
            try: write_XYT(XYT_path, xy, thresh)
            except Exception: print("File XYT_config Error !!!")
            dp, img = calibration.calibration_map(img, xy, thresh)
        # 检测模式 按键:"3"
        if action == ord('3'): model = 'Detection'
        # 重选颜色 按键:"4"
        if action == ord('4'):
            color_name = None
            init = 0
            color_list = {}
        # 退出标定 按键:"q"
        if action == ord('q'): model = 'Cancel'
        if model == 'Calibration': _, img = calibration.calibration_map(img, xy, thresh)
        if model == 'Cancel':
            dp = []
            msg = {}
        if len(dp) != 0: img = calibration.Perspective_transform(dp, img)
        if len(dp) != 0 and model == 'Detection':
            # 颜色选择&判断逻辑
            if len(color_list) == 0: threading.Thread(target=get_color_name, ).start()
            if len(color_list) != 0: img, msg = target.select_color(img, color_hsv, color_list)
        # 确认抓取 按键:"空格"
        if action == 32 and len(msg) != 0:
            threading.Thread(target=target.target_run, args=(msg, xy,)).start()
        # 设置随机颜色
        color = [[random.randint(0, 255) for _ in range(3)] for _ in range(255)]
        # 添加文字
        cv.putText(img, txt1, (0, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, color[random.randint(0, 254)], 1)
        cv.putText(img, txt2, (0, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, color[random.randint(0, 254)], 1)
        cv.putText(img, txt3, (0, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, color[random.randint(0, 254)], 1)
        cv.imshow("img", img)
        # 退出程序 按键:"esc"
        if action == 27:
            capture.release()
            cv.destroyAllWindows()
            break
    capture.release()
    cv.destroyAllWindows()
