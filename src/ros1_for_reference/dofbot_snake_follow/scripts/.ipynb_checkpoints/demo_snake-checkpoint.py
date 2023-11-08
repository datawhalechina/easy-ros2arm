# !/usr/bin/env python
# coding: utf-8
import threading
from dofbot_config import *
from snake_target import snake_target
from snake_ctrl import snake_ctrl
# 创建实例
snake_target = snake_target()
snake_ctrl = snake_ctrl()
# HSV参数路径
HSV_path = "HSV_config.txt"
# 初始化文字
txt1 = 'Button-control :'
txt2 = 'r ~ red, g ~ green, b ~ blue,'
txt3 = 'y ~ yellow, w ~ write, q&esc ~ quit'
# 设置随机颜色
color = [[random.randint(0, 255) for _ in range(3)] for _ in range(255)]
color_hsv = {"red": ((0, 25, 90), (10, 255, 255)),
             "green": ((53, 36, 40), (80, 255, 255)),
             "blue": ((116, 80, 90), (130, 255, 255)),
             "yellow": ((25, 20, 55), (50, 255, 255))}
if __name__ == '__main__':
    # 读取更新后的HSV参数
    try: read_HSV(HSV_path, color_hsv)
    except Exception: print("No file!!!")
    # 打开摄像头
    capture = cv.VideoCapture(0)
    # capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    # capture.set(cv.CAP_PROP_BRIGHTNESS, 30) #设置亮度 -64 - 64  0.0
    # capture.set(cv.CAP_PROP_CONTRAST, 50)   #设置对比度 -64 - 64  2.0
    # capture.set(cv.CAP_PROP_EXPOSURE, 156)  #设置曝光值 1.0 - 5000  156.0
    color_name = None
    while capture.isOpened():
        _, img = capture.read()
        action = cv.waitKey(10) & 0xff
        if action==ord('r'): color_name='red'
        if action==ord('g'): color_name='green'
        if action==ord('b'): color_name='blue'
        if action==ord('y'): color_name='yellow'
        # 获得运动信息
        img, snake_msg = snake_target.target_run(img, color_hsv)
        if len(snake_msg) == 1and color_name != None:
            threading.Thread(target=snake_ctrl.snake_main, args=(color_name, snake_msg,)).start()
        # 按键q或esc,退出程序
        if action == ord('q') or action == 27:
            cv.destroyAllWindows()
            capture.release()
            break
        # 添加文字
        cv.putText(img, color_name, (int(img.shape[0] / 2)+30, 30), cv.FONT_HERSHEY_SIMPLEX, 1,
                   color[random.randint(0, 254)], 2)
        cv.putText(img, txt1, (10, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv.putText(img, txt2, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv.putText(img, txt3, (10, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv.imshow("img", img)
    cv.destroyAllWindows()
    capture.release()
