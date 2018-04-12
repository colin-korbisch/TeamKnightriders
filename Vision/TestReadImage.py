import numpy as np
import cv2
import sys

def nothing(x):
	pass


imgini = cv2.imread("SampleRoadImg.PNG")
cv2.namedWindow('mask')

imgSmall = cv2.resize(imgini,(150,150),cv2.INTER_AREA)
img=cv2.cvtColor(imgSmall,cv2.COLOR_BGR2HSV)
display = img


cv2.namedWindow("Road",cv2.WINDOW_NORMAL)
cv2.resizeWindow("Road",300,300)

lowboundsYellow = np.array([10,59,67])
upboundsYellow = np.array([39,229,219])

lbPurp = np.array([141,18,97])
ubPurp = np.array([170,112,143])

maskY = cv2.inRange(display, lowboundsYellow, upboundsYellow)
maskP = cv2.inRange(display, lbPurp, ubPurp)

maskTot = cv2.bitwise_or(maskY,maskP)

res = cv2.bitwise_and(imgSmall,imgSmall, mask=maskTot)

cv2.imshow('Road',imgSmall)
cv2.imshow('mask',maskTot)
cv2.imshow('res',res)

cv2.waitKey(0)



