#!/usr/bin/env python
# coding: utf-8
from dofbot_config import *
from face import face_follow
follow = face_follow()
# 初始化窗口名称
img_name = "img"
# 初始化文字
txt1 = 'Button-control:'
txt2 = 'space ~ write'
txt3 = 'q&esc ~ quit'

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
        img = cv.resize(img, (640, 480))
        action = cv.waitKey(10) & 0xff
        cv.line(img, (320, 0), (320, 480), (255, 0, 0), 1)
        cv.line(img, (0, 240), (680, 240), (255, 0, 0), 1)
        # 添加文字
        cv.putText(img, txt1, (10, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv.putText(img, txt2, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv.putText(img, txt3, (10, 45), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        img = follow.follow_function(img)
        cv.imshow(img_name, img)
        if action == ord('q') or action == 27:
            cv.destroyAllWindows()
            capture.release()
            break
    cv.destroyAllWindows()
    capture.release()
