#!/usr/bin/env python
# coding: utf-8
import random
from color_follow import color_follow

# 创建实例
follow = color_follow()
# 初始化HSV_learning值
HSV_config = ()
# 初始化模式
model = 'Learning'
# 初始化颜色名称
color_name = 'red'
# 初始化窗口名称
img_name = "img"
# 初始化文字
txt1 = 'Button-control:'
txt2 = 'space ~ OK'
txt3 = 'l ~ learning'
txt4 = 'q&esc ~ quit'
# 设置随机颜色
color = [[random.randint(0, 255) for _ in range(3)] for _ in range(255)]
# 读取更新后的HSV参数
try: read_HSV(HSV_path, color_hsv)
except Exception: print("No HSV_config file!!!")

if __name__ == '__main__':
    # 打开摄像头
    capture = cv.VideoCapture(0)
    capture.set(3, 640)
    capture.set(4, 480)
    capture.set(5, 30)  # 设置帧率
    # 摄像头参数设置
    # capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    # capture.set(cv.CAP_PROP_BRIGHTNESS, 30) #设置亮度 -64 - 64  0.0
    # capture.set(cv.CAP_PROP_CONTRAST, 50) #设置对比度 -64 - 64  2.0
    # capture.set(cv.CAP_PROP_EXPOSURE, 156) #设置曝光值 1.0 - 5000  156.0
    while capture.isOpened():
        _, img = capture.read()
        img = cv.resize(img, (640, 480), )
        cv.line(img, (320, 0), (320, 480), (255, 0, 0), 1)
        cv.line(img, (0, 240), (680, 240), (255, 0, 0), 1)
        # 添加文字
        cv.putText(img, txt1, (10, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv.putText(img, txt2, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv.putText(img, txt3, (10, 45), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv.putText(img, txt4, (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        action = cv.waitKey(10) & 0xff
        if model == 'Learning':
            img, _ = follow.get_hsv(img)
            if action == 32:
                img, HSV_config = follow.get_hsv(img)
                model = 'Follow'
        if model == 'Follow':
            img= follow.learning_follow(img, HSV_config)
            if action == ord('l'): model = 'Learning'
        cv.imshow(img_name, img)
        # 按键q或esc,退出程序
        if action == ord('q') or action == 27:
            cv.destroyAllWindows()
            capture.release()
            break
    cv.destroyAllWindows()
    capture.release()
