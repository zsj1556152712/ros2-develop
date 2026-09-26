# -*- coding: utf-8 -*-
"""
视觉任务：OpenCV 轮廓检测演示
- 自己生成两张测试图：几何图形图 与 简单"表情包"圆脸
- 通过灰度化 -> 二值化 -> findContours -> drawContours 完成轮廓检测与绘制
"""
import os

import cv2
import numpy as np

# 输出目录固定为脚本所在目录，跨平台可移植
BASE = os.path.dirname(os.path.abspath(__file__))


def make_shape_image():
    """生成白底几何图形测试图：圆形、矩形、三角形、五角星、椭圆"""
    img = np.full((480, 640, 3), 255, np.uint8)
    # 圆形
    cv2.circle(img, (110, 120), 60, (30, 30, 30), -1)
    # 矩形
    cv2.rectangle(img, (200, 60), (330, 180), (30, 30, 30), -1)
    # 三角形
    pts = np.array([[520, 60], [420, 180], [620, 180]], np.int32)
    cv2.fillPoly(img, [pts], (30, 30, 30))
    # 五角星
    star = np.array([
        [200, 280], [230, 360], [310, 360], [245, 410], [275, 490],
        [200, 435], [125, 490], [155, 410], [90, 360], [170, 360]
    ], np.int32)
    cv2.fillPoly(img, [star], (30, 30, 30))
    # 椭圆
    cv2.ellipse(img, (520, 340), (90, 50), 0, 0, 360, (30, 30, 30), -1)
    return img


def make_meme_face():
    """生成一张简单的表情包风格图：黄色圆脸 + 五官"""
    img = np.full((480, 480, 3), 255, np.uint8)
    # 脸（深橙色，灰度<127 保证二值化后成为前景）
    cv2.circle(img, (240, 240), 180, (0, 100, 160), -1)
    # 眼、嘴用背景色（白色），在脸上形成"孔洞"，可被检测为内轮廓
    cv2.ellipse(img, (180, 210), (28, 38), 0, 0, 360, (255, 255, 255), -1)
    cv2.ellipse(img, (300, 210), (28, 38), 0, 0, 360, (255, 255, 255), -1)
    cv2.ellipse(img, (240, 300), (70, 55), 0, 0, 360, (255, 255, 255), -1)
    # 腮红
    cv2.circle(img, (130, 300), 30, (255, 120, 150), -1)
    cv2.circle(img, (350, 300), 30, (255, 120, 150), -1)
    return img


def detect_and_draw(src_img, title, out_path, mode=cv2.RETR_EXTERNAL):
    """对一张图做轮廓检测并保存对比图（左：原图；右：轮廓绘制结果）
    mode: RETR_EXTERNAL 只取最外层轮廓；RETR_LIST 取所有轮廓（含孔洞）"""
    gray = cv2.cvtColor(src_img, cv2.COLOR_BGR2GRAY)
    # 高斯模糊降噪，避免噪声产生过多细小轮廓
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    # 二值化：前景（深色图形）为 255，背景为 0
    _, binary = cv2.threshold(blur, 127, 255, cv2.THRESH_BINARY_INV)

    # 轮廓检测，简化存储以减少点数
    contours, _ = cv2.findContours(binary, mode, cv2.CHAIN_APPROX_SIMPLE)

    # 在原图上绘制轮廓（绿色，2 像素宽）
    result = src_img.copy()
    cv2.drawContours(result, contours, -1, (0, 200, 0), 2)

    # 标注轮廓数量
    cv2.putText(result, "contours: %d" % len(contours), (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 200), 2)

    # 拼接对比图：左原图、右结果
    h, w = src_img.shape[:2]
    canvas = np.full((h, w * 2 + 10, 3), 255, np.uint8)
    canvas[:, :w] = src_img
    canvas[:, w + 10:] = result
    cv2.putText(canvas, "original", (10, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 100, 100), 2)
    cv2.putText(canvas, "contour result", (w + 20, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 160, 0), 2)

    cv2.imwrite(out_path, canvas)
    print(title, "->", out_path, "| 检测到轮廓数:", len(contours))


if __name__ == "__main__":
    shape = make_shape_image()
    detect_and_draw(shape, "几何图形", os.path.join(BASE, "vision_contour_shapes.png"))

    face = make_meme_face()
    # 表情包图用 RETR_LIST：同时勾出脸的外轮廓与眼、嘴的孔洞轮廓
    detect_and_draw(face, "表情包圆脸", os.path.join(BASE, "vision_contour_meme.png"), mode=cv2.RETR_LIST)
