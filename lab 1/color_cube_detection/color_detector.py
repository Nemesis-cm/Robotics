import cv2
import numpy as np

img = cv2.imread("img57.jpg")

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
img_filt = cv2.medianBlur(img, 9)

green_lower = np.array([5, 5, 5])
green_upper = np.array([120, 180, 120])

yellow_lower = np.array([10,160,160])
yellow_upper = np.array([75,255,255])

mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)
mask_green = cv2.inRange(hsv, green_lower, green_upper)

cv2.imshow("Mask", mask_green)
cv2.imshow("Mask", mask_yellow)

