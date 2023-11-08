# !/usr/bin/env python
# coding: utf-8
import cv2 as cv
from color_follow import color_follow

Arm_color_follow = color_follow()
img_name = "img"
color_name = "red"
# 初始化HSV值
color_hsv = {"red": ((0, 157, 132), (8, 255, 255)),
             "green": ((55, 131, 84), (78, 255, 255)),
             "blue": ((110, 151, 189), (120, 255, 255)),
             "yellow": ((24, 134, 180), (30, 231, 255))}
capture = cv.VideoCapture(0)
capture.set(3, 640)
capture.set(4, 480)
capture.set(5, 30)  # 设置帧率
while capture.isOpened():
    ret, img = capture.read()
    img = cv.resize(img, (640, 480))
    cv.line(img, (320, 0), (320, 480), (255, 0, 0), 1)
    cv.line(img, (0, 240), (680, 240), (255, 0, 0), 1)
    action = cv.waitKey(10) & 0xff
    color_name = "red"
    img = Arm_color_follow.follow_function(img, color_name, color_hsv[color_name])
    cv.imshow(img_name, img)
    # 按键q或esc,退出程序
    if action == ord('q') or action == 27:
        cv.destroyAllWindows()
        capture.release()
        break
cv.destroyAllWindows()
capture.release()
