from get_qr import get_qrcode_range
import cv2

img = cv2.imread("qr1.png")

img1,img2 = get_qrcode_range(img)
cv2.imshow('img1',img1)
cv2.imshow('img2',img2)
cv2.waitKey(0)