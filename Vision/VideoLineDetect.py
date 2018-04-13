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


# cv2.namedWindow('image')
# cv2.resizeWindow('image',300,300)
# cv2.createTrackbar('Hue', 'image', 61, 179, nothing)
# cv2.createTrackbar('Sat', 'image', 235, 255, nothing)
# cv2.createTrackbar('Val', 'image', 255, 255, nothing)
# cv2.createTrackbar('HueU', 'image', 61, 179, nothing)
# cv2.createTrackbar('SatU', 'image', 235, 255, nothing)
# cv2.createTrackbar('ValU', 'image', 255, 255, nothing)
 

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
	# hue = cv2.getTrackbarPos('Hue', 'image')
	# sat = cv2.getTrackbarPos('Sat', 'image')
	# val = cv2.getTrackbarPos('Val', 'image')
	# hueU = cv2.getTrackbarPos('HueU', 'image')
	# satU = cv2.getTrackbarPos('SatU', 'image')
	# valU = cv2.getTrackbarPos('ValU', 'image')

	# lowboundsYellow = np.array([hue,sat,val])
	# upboundsYellow = np.array([hueU,satU,valU])
	# NEW BOUNDS FOUND FROM VIDEO:
	
	lowboundsYellow = np.array([25,29,60])
	upboundsYellow = np.array([28,236,255])
	lbPurp = np.array([141,18,97])
	ubPurp = np.array([170,112,143])

	maskY = cv2.inRange(display, lowboundsYellow, upboundsYellow)
	maskP = cv2.inRange(display, lbPurp, ubPurp)

	maskTot = cv2.bitwise_or(maskY,maskP)

	res = cv2.bitwise_and(imgSmall,imgSmall, mask=maskTot)

	gray = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)
	edges = cv2.Canny(gray,50,150,apertureSize = 3)
	lines = []
	lines = cv2.HoughLines(edges,0.5,np.pi/180,36)
	if np.any(lines):
		for line in lines:
			line = line[0]
			rho = line[0]
			theta = line[1]
			print theta*180/np.pi
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


	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
	
	# if the `q` key was pressed, break from the loop
	if key == 27:
		cv2.waitKey(0)
		break