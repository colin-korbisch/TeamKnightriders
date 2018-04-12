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

gray = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray,50,150,apertureSize = 3)

lines = cv2.HoughLines(edges,0.5,np.pi/180,39)

for line in lines:
	line = line[0]
	rho = line[0]
	theta = line[1]
	print theta
	a = np.cos(theta)
	b = np.sin(theta)
	x0 = a*rho
	y0 = b*rho
	x1 = int(x0 + 100*(-b))
	x2 = int(x0 - 100*(-b))
	y1 = int(y0 + 100*a)
	y2 = int(y0 - 100*a)
	color_horz = [0,0,255]
	color_lane = [255,0,0]
	if abs(theta) <= np.pi/2 + 0.1 and abs(theta) >= np.pi/2 - 0.1:
		colorLine = color_horz
	else:
		colorLine = color_lane
	cv2.line(imgSmall,(x1,y1),(x2,y2),colorLine,2)

#linesP = cv2.HoughLinesP(maskTot,1,np.pi/180,50,30,6)
#for line in linesP:
#	line = line[0]
#	x1 = line[0]
#	y1 = line[1]
#	x2 = line[2]
#	y2 = line[3]
#	theta = np.arctan2(y2-y1,x2-x1)
#	print theta
#	cv2.line(imgSmall,(x1,y1),(x2,y2),(0,255,0),1)

	


cv2.imshow('Road',imgSmall)
cv2.imshow('mask',maskTot)
cv2.imshow('res',res)
cv2.imshow('edge',edges)

cv2.waitKey(0)



