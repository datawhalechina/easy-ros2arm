#!/usr/bin/env python
# coding: utf-8
# 导入头文件
from color_sorting import color_sorting
import cv2 as cv
# 创建实例
sorting = color_sorting()
# HSV参数路径
HSV_path = "HSV_config.txt"
# 颜色HSV阈值
color_hsv = {"red": ((0, 197, 122), (6, 255, 186)),
             "green": ((53, 50, 45), (74, 255, 153)),
             "blue": ((116, 174, 173), (123, 201, 201)),
             "yellow": ((25, 146, 172), (30, 225, 214))}

if __name__ == '__main__':
    # 读取更新后的HSV参数
    try: read_HSV(HSV_path, color_hsv)
    except Exception: print("No HSV_config file!!!")
    # 打开摄像头
    capture = cv.VideoCapture(0)
    # 当摄像头正常打开的情况下,循环读取每一帧
    while capture.isOpened():
        _, img = capture.read()
        # 重置图像大小
        img = cv.resize(img, (640, 480), )
        # 调用['你放我抓']抓取函数
        img = sorting.Sorting_grap(img, color_hsv)
        action = cv.waitKey(10) & 0xff
        # 显示图像
        cv.imshow("img", img)
        # 按 q 或 esc 退出
        if action == ord('q') or action == 27:
            cv.destroyAllWindows()
            capture.release()
            break
    cv.destroyAllWindows()
    capture.release()
