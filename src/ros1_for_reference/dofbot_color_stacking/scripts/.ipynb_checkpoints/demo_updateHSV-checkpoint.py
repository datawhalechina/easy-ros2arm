# !/usr/bin/env python
# coding: utf-8
import random
from dofbot_config import *

# 创建HSV更新实例
hsv_update = update_hsv()
# 创建HSV更新滑动条实例
trackbar = HSV_Trackbar()
# HSV参数路径
HSV_path = "HSV_config.txt"
# 初始化颜色名称
hsv_name = 'red'
# 初始化窗口名称
img_name = "img"
# 初始化文字
txt1 = 'Button-control:'
txt2 = 'r ~ red,g ~ green,b ~ blue,'
txt3 = 'y ~ yellow,w ~ write,q&esc ~ quit'
# 初始化HSV值
color_hsv = {"red": ((0, 99, 89), (3, 255, 255)),
             "green": ((69, 166, 86), (74, 255, 133)),
             "blue": ((113, 125, 163), (123, 255, 233)),
             "yellow": ((28, 147, 183), (35, 255, 254))}

if __name__ == '__main__':
    # 读取更新后的HSV参数
    try: read_HSV(HSV_path, color_hsv)
    except Exception: print("No HSV_config file!!!")
    # 打开摄像头
    capture = cv.VideoCapture(0)
    # 创建滑动条
    trackbar.create_bar(img_name, hsv_name, color_hsv)
    while capture.isOpened():
        # 读取图片
        _, img = capture.read()
        # 重置图像大小
        img = cv.resize(img, (640, 480), )
        action = cv.waitKey(10) & 0xff
        if action == 27 or action == ord('q'):
            capture.release()
            cv.destroyAllWindows()
            break
        if action == ord("r"):
            hsv_name = "red"
            cv.destroyAllWindows()
            trackbar.create_bar(img_name, hsv_name, color_hsv)
        if action == ord("g"):
            hsv_name = "green"
            cv.destroyAllWindows()
            trackbar.create_bar(img_name, hsv_name, color_hsv)
        if action == ord("b"):
            hsv_name = "blue"
            cv.destroyAllWindows()
            trackbar.create_bar(img_name, hsv_name, color_hsv)
        if action == ord("y"):
            hsv_name = "yellow"
            cv.destroyAllWindows()
            trackbar.create_bar(img_name, hsv_name, color_hsv)
        if action == ord('w'):
            # print('写入配置文件')
            try: write_HSV(HSV_path,color_hsv)
            except Exception: print("File Path Error!!!")
            # 所有颜色取反
            cv.bitwise_not(img, img)
        # 实时获取滑动条的值
        color_hsv[hsv_name] = trackbar.get_range()
        # 获取检测后的彩图和二值图
        img, binary = hsv_update.get_contours(img, hsv_name, color_hsv[hsv_name], color_hsv)
        # 设置一个窗口显示多幅图
        imgs = ManyImgs(0.5, [img, binary])
        # 设置随机颜色
        color = [[random.randint(0, 255) for _ in range(3)] for _ in range(255)]
        # 添加文字
        cv.putText(imgs, hsv_name, (int(imgs.shape[0] / 2), 30), cv.FONT_HERSHEY_SIMPLEX, 1,
                   color[random.randint(0, 254)], 2)
        cv.putText(imgs, txt1, (int(imgs.shape[0] / 2) * 3 - 30, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv.putText(imgs, txt2, (int(imgs.shape[0] / 2) * 3 - 30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv.putText(imgs, txt3, (int(imgs.shape[0] / 2) * 3 - 30, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv.imshow(img_name, imgs)
    capture.release()
    cv.destroyAllWindows()





