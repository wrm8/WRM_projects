#!/usr/bin/env python3
import cv2

# 打开摄像头
cap = cv2.VideoCapture(0)

# 检查摄像头是否成功打开
if not cap.isOpened():
    print("错误：无法打开摄像头")
    exit()

# 循环显示摄像头图像
while True:
    # 读取一帧图像
    ret, frame = cap.read()
    
    # 检查是否成功读取图像
    if not ret:
        print("错误：无法获取图像")
        break
    
    # 显示图像
    cv2.imshow('Camera', frame)
    
    # 等待按键，如果按下 'q' 键则退出
    # 也可以按 's' 键保存当前图像
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print("退出程序")
        break
    elif key == ord('s'):
        # 保存当前图像
        cv2.imwrite("//home//jetson//catkin_ws//picture//photo.png", frame)
        print("照片已保存: //home//jetson//catkin_ws//picture//photo.png")

# 释放摄像头资源
cap.release()
# 关闭所有OpenCV窗口
cv2.destroyAllWindows()