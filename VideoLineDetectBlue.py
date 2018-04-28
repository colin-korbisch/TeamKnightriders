#VIDEO LINE DETECT

# import the necessary packages
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2


def nothing(*arg):
	pass 
# initialize the camera and grab a reference to the raw camera capture

camera = PiCamera()
camera.resolution = (1280, 720)
camera.framerate = 60
rawCapture = PiRGBArray(camera, size=(1280, 720))


cv2.namedWindow('image')
cv2.resizeWindow('image',300,300)
cv2.createTrackbar('Hue', 'image', 141, 255, nothing)
cv2.createTrackbar('Sat', 'image', 18, 255, nothing)
cv2.createTrackbar('Val', 'image', 97, 255, nothing)
cv2.createTrackbar('HueU', 'image', 170, 179, nothing)
cv2.createTrackbar('SatU', 'image', 250, 255, nothing)
cv2.createTrackbar('ValU', 'image', 255, 255, nothing)
 

# allow the camera to warmup
time.sleep(0.1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	key = cv2.waitKey(5) & 0xFF
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	imgini = frame.array
 
	# show the frame
 
	cv2.namedWindow('mask')
	#rows,cols,_ = imageini.shape
	#M = cv2.getRotationMatrix2D((cols/2,rows/2),-90,1)
	#imgini = cv2.warpAffine(imageini,M,(cols,rows))

	imgSmall = cv2.resize(imgini,(150,150),cv2.INTER_AREA)
	img=cv2.cvtColor(imgSmall,cv2.COLOR_BGR2HSV)
	display = img


	cv2.namedWindow("Road",cv2.WINDOW_NORMAL)
	cv2.resizeWindow("Road",300,300)
	# NEW BOUNDS FOUND FROM VIDEO:
	
	# lowboundsYellow = np.array([15,51,34])
	# upboundsYellow = np.array([34,248,220])
	lbBlue = np.array([7,61,58])
	ubBlue = np.array([115,222,255])

	# lbPurp = np.array([141,18,97])
	# ubPurp = np.array([170,250,255])
	# lbPurp = np.array([hue,sat,val])
	# ubPurp = np.array([hueU,satU,valU])

	# maskY = cv2.inRange(display, lowboundsYellow, upboundsYellow)
	# maskP = cv2.inRange(display, lbPurp, ubPurp)
	maskB = cv2.inRange(display,lbBlue,ubBlue)
	TopLine = maskB[0,0:75]
	print "TopLine Length:", len(TopLine)
	if np.any(TopLine==255):
		dist = np.max(np.where(TopLine==255))
	print "Left Lane Distance: ",dist

	# maskTot = cv2.bitwise_or(maskY,maskP)

	res = cv2.bitwise_and(imgSmall,imgSmall, mask=maskB)

	gray = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)
	edges = cv2.Canny(gray,50,150,apertureSize = 3)
	lines = []
	lines = cv2.HoughLines(edges,0.5,np.pi/180,36)
	Llines = cv2.HoughLines(edges[:,0:75],0.5,np.pi/180,20)
	Rlines = cv2.HoughLines(edges[:,76:150],0.5,np.pi/180,20)
	cv2.line(imgSmall,(75,0),(75,150),[0,0,255],2)
	# print "Left Lines:", Llines
	# print "Right Lines:", Rlines

	if np.any(Llines):
		for line in Llines:
			line = line[0]
			# print "Left Lane Angle:", line[1]

	if np.any(Rlines):
		for line in Rlines:
			line = line[0]
			# print "Right Lane Angles", line[1]
	# if np.any(lines):
	# 	for line in lines:
	# 		line = line[0]
	# 		rho = line[0]
	# 		theta = line[1]
	# 		print theta*180/np.pi
	# 		a = np.cos(theta)
	# 		b = np.sin(theta)
	# 		x0 = a*rho
	# 		y0 = b*rho
	# 		x1 = int(x0 + 100*(-b))
	# 		x2 = int(x0 - 100*(-b))
	# 		y1 = int(y0 + 100*a)
	# 		y2 = int(y0 - 100*a)
	# 		color_horz = [0,0,255]
	# 		color_R = [255,0,0]
	# 		color_L = [0,255,0]
	# 		if abs(theta) <= np.pi/2 + 0.005 and abs(theta) >= np.pi/2 - 0.005:
	# 			colorLine = color_horz #horz line should be red line
	# 		elif abs(theta) >= 0 and abs(theta) <= np.pi/8:
	# 			colorLine = color_L
	# 		elif abs(theta) >= 7*np.pi/8 and abs(theta) <= np.pi:
	# 			colorLine = color_R
	# 		else:
	# 			colorLine = [255,255,0]
	cv2.line(imgSmall,(0,0),(149,0),[255,0,0],2)
	cv2.line(imgSmall,(0,143),(149,143),[255,255,0],2)

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
	cv2.imshow('mask',maskB)
	cv2.imshow('res',res)
	cv2.imshow('edge',edges)


	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
	
	# if the `q` key was pressed, break from the loop
	if key == 27:
		cv2.waitKey(0)
		break