from qrcode_yolo3.get_qr import get_qrcode_range
import cv2
from pyzbar.pyzbar import decode, ZBarSymbol
import cv2
import pyzbar.pyzbar as pyzbar
import numpy as np

import zxing

def histogram_equalization(image):
    # 計算灰度直方圖
    hist = [0] * 256
    total_pixels = image.shape[0] * image.shape[1]

    for i in range(image.shape[0]):
        for j in range(image.shape[1]):
            pixel_value = image[i, j]
            hist[pixel_value] += 1
    # 計算CDF
    cdf = [0] * 256
    cdf[0] = hist[0] / total_pixels

    for i in range(1, 256):
        cdf[i] = cdf[i-1] + hist[i] / total_pixels
    
    # 將CDF映射到新的灰度值範圍
    cdf_normalized = [int(round(value * 255)) for value in cdf]

    # 將灰度值映射到新的範圍
    equalized_image = image.copy()

    for i in range(image.shape[0]):
        for j in range(image.shape[1]):
            pixel_value = image[i, j]
            equalized_image[i, j] = cdf_normalized[pixel_value]

    return equalized_image

img = cv2.imread("qrcode_yolo3/qr1.png")
h, w = img.shape[:2]
# print(h,w)
# image = cv2.resize(img,(w,h), interpolation=cv2.INTER_AREA)
# min_val = np.min(image)
# max_val = np.max(image)
# stretched_image = (255 / (max_val - min_val)) * (image - min_val)
# stretched_image = stretched_image.astype(np.uint8)
# stretched_image = histogram_equalization(image)

# h, w = img.shape[:2]
# print(h,w)
# print(type(img))
img1,img2,pos = get_qrcode_range(img)
# print(type(img1))
# print(img1.shape)
# # print(pos)
# decodeDisplay(img)

cv2.imshow('img1',img1)
cv2.imwrite('img1.jpg',img1)
cv2.imshow('img2',img2)
cv2.imwrite('img2.jpg',img2)
# # # cv2.imshow('img2',img2)
# img1 = cv2.GaussianBlur(img, (13,13), 1)
barcodes = decode(img1,symbols=[ZBarSymbol.QRCODE])
print(barcodes)
cv2.waitKey(0)
