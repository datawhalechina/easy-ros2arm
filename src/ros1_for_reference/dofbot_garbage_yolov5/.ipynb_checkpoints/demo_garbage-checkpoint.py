#!/usr/bin/env python
# coding: utf-8
import random
import threading
from dofbot_config import *
from garbage_identify import garbage_identify

# 创建获取目标实例
target = garbage_identify()
# 创建相机标定实例
calibration = Arm_Calibration()
# 创建滑动条实例
calibration_trackbar = calibration_Trackbar()
index=0
# 初始化标定方框边点
dp = []
# 初始化抓取信息
msg = {}
# 初始化模式
model = None
# 初始化1,2舵机角度值
xy = [90, 135]
# 初始化二值图阈值
thresh = 130
# 初始化文字
txt0 = 'Model-Loading...'
txt1 = 'Button-control: '
txt2 = '1 ~ Calibration, 2 ~ CalOK, 3 ~ Detection,'
txt3 = 'q ~ CalCancel, Space ~ Grap, esc ~ Quit'
# XYT参数路径
XYT_path = "XYT_config.txt"
# 设置随机颜色
color = [[random.randint(0, 255) for _ in range(3)] for _ in range(255)]
if __name__ == '__main__':
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
            except Exception: print("File XYT_config Error!!!")
            dp, img = calibration.calibration_map(img, xy, thresh)
        # 检测模式 按键:"3"
        if action == ord('3'): model = 'Detection'
        # 退出标定 按键:"q"
        if action == ord('q'): model = 'Cancel'
        if model == 'Calibration': _, img = calibration.calibration_map(img, xy, thresh)
        if model == 'Cancel':
            dp = []
            msg = {}
        if len(dp) != 0: img = calibration.Perspective_transform(dp, img)
        if len(dp) != 0 and model == 'Detection':
            img, msg = target.garbage_run(img)
        # 确认抓取 按键:"空格"
        if action == 32 and len(msg) != 0:
            threading.Thread(target=target.garbage_grap, args=(msg, xy,)).start()
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
