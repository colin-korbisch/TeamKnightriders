from BreadthSearch import *
from PiSkyPixy import *
import RPi.GPIO as GPIO
from picamera.array import PiRGBArray
from picamera import PiCamera
from imutils.video.pivideostream import PiVideoStream
import imutils
import time
import cv2


# global camera
# global rawCapture
# global DEVICE_ADDRESS
# global isintersect
# global dp
# global sp
# global radio
# class ROBOT
# 	def __init__():
# 	        self.radio = radioSetup()
# 	        self.camera = PiCamera()
# 	        self.camera.resolution = (640, 360)
# 			self.camera.framerate = 30
# 			self.rawCapture = PiRGBArray(camera, size=(640, 360))
# 	        self.DEVICE_ADDRESS = 0x54


# Robot Test Run
def interDetect(mask,row,isintersect):
	interLine = mask[row]
	findYellow = np.where(np.equal(interLine,255))
	# print "Blue Px Loc:", findYellow
	if findYellow[0].size:
		findLine = np.amax(findYellow)
	else:
		findLine = 0
	if findLine < 75:
		# print "Seeing Left Lane"
		isintersect = True
	elif findLine > 75 and isintersect:
		interLineTop = mask[row-8]
		# interLineBot = mask[row+5]
		fLineT = np.where(np.equal(interLineTop,255))
		# fLineB = np.where(np.equal(interLineBot,255))
		if fLineT[0].size:
			findYelTop = np.amax(fLineT)
		else:
			findYelTop = 150
		# if fLineB[0].size:
			# findYelBot = np.amax(fLineB)
		# else:
			# findYelBot = 150
		if findYelTop < 75: #and findYelBot < 75:
			isintersect = True
		else:
			isintersect = False
	return isintersect
def GetErrorSignal(theta_out,distance,leftBool):
	# if np.any(theta_out >= np.pi/2):
	# 	theta_out = theta_out - np.pi

	# setpointL = 2*np.pi/180
	# setpointR = 168*np.pi/180
	setpointL = 1.5*np.pi/180
	setpointR = 169.5*np.pi/180
	setLDist =  40


	LError = []
	RError = []

	if not leftBool:
		#Find right lines:
		arr1 = np.greater_equal(theta_out,7*np.pi/8)
		arr2 = np.less_equal(theta_out,np.pi)
		and_arr = np.logical_and(arr1,arr2)
		rightLineIDs = np.where(and_arr)

		# Right Error:
		if rightLineIDs[0].size and not isinstance(rightLineIDs,tuple):
			RError = setpointR - np.mean(theta_out[rightLineIDs])
		else:
			RError = None

		return RError
	else:
			#Find left lines:
		LDistE = setLDist - distance
		arr1 = np.greater_equal(theta_out,0)
		arr2 = np.less_equal(theta_out,3*np.pi/16)
		and_arr = np.logical_and(arr1,arr2)
		leftLineIDs = np.where(and_arr)
		# print leftLineIDs

		# Left Error:
		if leftLineIDs[0].size and not isinstance(leftLineIDs,tuple):
			LError = setpointL - np.mean(theta_out[leftLineIDs])
		else:
			LError = 0.005
			UpdateMotor(7.88)
		return LError, LDistE

def StraightLineControl(EsigHistory,LaneDist):
	# Define PID control constants:
	# Angle PID
	kp = 0.25
	kd = 3.2
	ki = 0.001

	# Lane Distance Controller:
	# lkp = 0.053
	# lkd = 0.0019
	# lki = 0.0022
	lkp = 0.056
	lkd = 0.0035
	lki = 0.0022

		# Finite Difference Derivative
	if len(EsigHistory) >=3:
		terms = EsigHistory[-3:]
		Edif = 1.5*terms[-1]-2*terms[-2]+0.5*terms[-3]
	elif len(EsigHistory) == 2:
		Edif = EsigHistory[-1] - EsigHistory[-2]
	else:
		Edif = EsigHistory[-1]
	Esum = np.trapz(EsigHistory[-10:])

	if len(LaneDist) >= 3:
		lterm = LaneDist[-3:]
		Ldif = 1.5*lterm[-1]-2*lterm[-2]+0.5*lterm[-3]
	else:
		Ldif = LaneDist[-1]
	# Integral Term Goes Here:
	Lsum = np.trapz(LaneDist[-10:])

	# steerContro = lkp*LaneDist[-1] + lkd*Ldif + lki*Lsum
	steerContro = 0.55*(kp*EsigHistory[-1] + kd*Edif + ki*Esum) + 0.45*(lkp*LaneDist[-1] + lkd*Ldif + lki*Lsum)

	UpdateSteering(10.8,steerContro)
	if 8-0.002*abs(LaneDist[-1]) <= 7.9:
		UpdateMotor(7.7)
		time.sleep(0.05)
		UpdateMotor(7.9)
	else:
		UpdateMotor(8-0.002*abs(LaneDist[-1]))

def GoStraight(skipIntersect,nxTurn):
	# continue going straight until you skip the correct num intersections
	# once you do, approach the intersection until you are at the correct distance
	# once at the correct distance, leave this state to begin your turn
	EsigHistory = []
	LdistHist = []
	EsignalM = 0
	LsigM = 0
	
	startTime = time.time()
	print "going straight"
	UpdateMotor(7.88)
	leave = False
	numInt = 0
	isintersect = False
	enterInt = False
	leaveInt = True
	oldInt = False

	# for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	while True:
		frame = vs.read()
		# Do some image processing stuff...
		oldInt = isintersect
		rho_out, theta_out, theta_outL,theta_outR, L_dist, isintersect = readFrame(frame, isintersect)
		# print "Intersection State:" ,isintersect
		# rawCapture.truncate(0)
		# Check for stop lights
		# NEED TO TIGHTEN-UP THIS CONDITION
		# if not isintersect:
		# 	signature = lookStopLight()

		# 	if sum(np.equal(signature,1)) > 3:
		# 		StopCar()
		# 		is_stopped = True
		# 	else:
		# 		UpdateMotor(8)
		# 		is_stopped = False
		# else:
		# 	UpdateMotor(8)
		# 	is_stopped = False

		# Counting out number of intersections
		print "Numb of Inter Passed:", numInt
		# print "Numb of Int to Skip: ", skipIntersect
		# print numInt == skipIntersect
		if numInt != skipIntersect:
			if isintersect and not oldInt:
				enterInt = True
				leaveInt = False
			if not isintersect and oldInt and enterInt:
				leaveInt = True
				enterInt = False
				numInt = numInt + 1
			# print "Did Enter Int?", enterInt
			# print "Did Leave Int?", leaveInt
		else:
			print "Entering Turn Mode"
			break

		if not isinstance(theta_outL,np.ndarray):
			theta_outL = np.array(theta_outL)
		if not isinstance(theta_outR,np.ndarray):
			theta_outR = np.array(theta_outR)
		if not isinstance(rho_out,np.ndarray):
			rho_out = np.array(rho_out)		

		LError, LDistE = GetErrorSignal(theta_outL,L_dist, 1)
		RError = GetErrorSignal(theta_outR, L_dist, 0)
		# if LError < 0:
		# 	LError = LError - 0.05

		# print "Left Lane Error:", LError
		# print "Right Lane Error:", RError
		# print "Lane Tape Error:", LDistE
		# Mean Error:
		if RError:
			ErrorSignal = (LError+RError)/2.
		else:
			ErrorSignal = LError

		curTime = time.time()
		# print oldE, Edif, Esum
		# oldEM  = (oldE+oldEM)/2.
		# EdifM = (EdifM+Edif)/2.
		# EsumM = (EsumM+Esum)/2.
		EsignalM = (EsignalM+ErrorSignal)/2.
		LsigM = (LsigM+LDistE)/2.

		if curTime-startTime >= 0.0125:
			EsigHistory.append(EsignalM)
			LdistHist.append(LsigM)
			StraightLineControl(EsigHistory,LdistHist)
			startTime = time.time()
		else:
			UpdateMotor(7.9)
			time.sleep(0.1)

		#Find horz lines:
		# arr1 = np.greater_equal(rotateTheta,7*np.pi/8)
		# arr2 = np.less_equal(rotateTheta,np.pi/8)
		# and_arr = np.logical_and(arr1,arr2)
		# horzLineIDs = np.where(and_arr)

def TurnRight():
	print "Right Turn State"
	EsigHistory = []
	LdistHist = []
	EsignalM = 0
	LsigM = 0
	RError = None

	UpdateMotor(7.8)
	isTurning = False
	lineSkip = True
	checkComplete = False
	startTime = time.time()

	# for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	while True:
		frame = vs.read()
		curTime = time.time()
		# Do some image processing stuff...

		rho_out, theta_out, theta_outL, theta_outR, L_dist, detectR = readFrameRight(frame)
		# print "Can we see the right lane?: ", detectR
		# rawCapture.truncate(0)
		if not isTurning:
			if detectR:
				print "Now Turning Right:"
				UpdateSteering(13,0)
				UpdateMotor(8.2)
				time.sleep(2.3)
				isTurning = True
			# if Interdistance >= 20: # How far the first intersection line travels down the frame BEFORE the next intersection line appears in frame
			# 	lineSkip = False

			# if distancePx >= 10 and not lineSkip:# Adjust this constant as needed
			# 	UpdateSteering(13)
			# 	isTurning = True

		if not isinstance(theta_out,np.ndarray):
			theta_out = np.array(theta_out)
		if not isinstance(rho_out,np.ndarray):
			rho_out = np.array(rho_out)		

		LError, LDistE = GetErrorSignal(theta_outL,L_dist, 1)
		# RError = GetErrorSignal(theta_outR, L_dist, 0)

		# Mean Error:
		if RError:
			ErrorSignal = (LError+RError)/2.
		else:
			ErrorSignal = LError
		
		EsignalM = (EsignalM+ErrorSignal)/2.
		LsigM = (LsigM+LDistE)/2.

		if not isTurning and curTime-startTime >= 0.0125:
			EsigHistory.append(EsignalM)
			LdistHist.append(LsigM)
			StraightLineControl(EsigHistory,LdistHist)
			startTime = time.time()

		if isTurning:
			# Continue turn until a line around 45deg crosses the frame
			# Now you need to check the frame until you complete the turn

			# Find 45deg lines
			arr1 = np.greater_equal(theta_outL,0)
			arr2 = np.less_equal(theta_outL, 3*np.pi/16)
			and_arr = np.logical_and(arr1,arr2)
			diagLineIDs = np.where(and_arr)
			if diagLineIDs[0].size:
				checkComplete = True

		if checkComplete:
			isTurning = False
			UpdateSteering(11,0)
			print "Turn Complete"
			break

def readFrame(frame, isintersect):

	rho_out = []
	theta_outL = []
	theta_outR = []
	theta_out = []
	intersectDetectRow = 142 # need to test this constant

	imgSmall = cv2.resize(frame,(150,150),cv2.INTER_AREA)
	img=cv2.cvtColor(imgSmall,cv2.COLOR_BGR2HSV)
	
	# lowboundsYellow = np.array([10,59,67])
	# upboundsYellow = np.array([39,229,219])

	# lowboundsYellow = np.array([15,51,34])
	# upboundsYellow = np.array([34,248,220])

	# lbPurp = np.array([141,18,97])
	# ubPurp = np.array([170,250,255])

	# maskY = cv2.inRange(img, lowboundsYellow, upboundsYellow)
	# maskP = cv2.inRange(img, lbPurp, ubPurp)
	# maskTot = cv2.bitwise_or(maskY,maskP)
	maskB = cv2.inRange(img,np.array([7,61,58]),np.array([115,222,255]))
	TopLine = maskB[0,0:75]
	if np.any(TopLine==255):
		distance = np.max(np.where(TopLine==255))
	else:
		distance = 75

	res = cv2.bitwise_and(imgSmall,imgSmall, mask=maskB)

	gray = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)

	edges = cv2.Canny(gray,50,150,apertureSize = 3)

	lines = cv2.HoughLines(edges,0.5,np.pi/180,35)
	leftLines = cv2.HoughLines(edges[:,0:70],0.5,np.pi/180,20)
	rightLines = cv2.HoughLines(edges[:,80:150],0.5,np.pi/180,20)
	if np.any(leftLines != None):
		for line in leftLines:
			line = line[0]
			rho = line[0]
			theta = line[1]
			rho_out.append(rho)
			theta_outL.append(theta)
	if np.any(rightLines != None):
		for line in rightLines:
			line = line[0]
			rho = line[0]
			theta = line[1]
			rho_out.append(rho)
			theta_outR.append(theta)
	if np.any(lines != None):
		for line in lines:
			line = line[0]
			rho = line[0]
			theta = line[1]
			rho_out.append(rho)
			theta_out.append(theta)

	# Intersection Detection

	# interLine = maskY[intersectDetectRow]
	# findLine = np.amax(np.where(np.equal(interLine,255)))
	# if findLine < 100:
	# 	isintersect = True
	# elif findLine > 100 and isintersect:
	# 	isintersect = False
	# l1 = interDetect(maskB,intersectDetectRow-1,isintersect)
	# l2 = interDetect(maskB,intersectDetectRow,isintersect)
	# l3 = interDetect(maskB,intersectDetectRow+1,isintersect)

	# New Intersection Detection
	ar1 = np.where(maskB[129,:]==255)
	if ar1[0].size:
		l1 = np.max(ar1)
		if l1<75:
			checkTop = True
		else:
			checkTop = False
	else:
		checkTop = False

	ar3 = np.where(maskB[132,:]==255)
	if ar3[0].size:
		l3 = np.max(ar3)
		if l3<75:
			checkMid = True
		else:
			checkMid = False
	else:
		checkMid = False
	l2 = np.max(np.where(maskB[139,:]==255))
	if l2<75:
		checkBot = True
	else:
		checkBot = False

	print "interChecK", checkTop, checkMid, checkBot

	if sum([checkTop,checkMid,checkBot]) > 1:
		isintersect = True
	else:
		isintersect = False

	# print "Intersection Detection: ", l1,l2,l3
	# if sum([l1,l2,l3]) > 1:
	# 	isintersect = True
	# # elif sum([l1,l2,l3]) < 1 and isintersect:
	# # 	isintersect = False
	# else:
	# 	isintersect = False

	# print "Left Lane Angles:", theta_outL
	# print "Right Lane Angles:", theta_outR
	return rho_out, theta_out, theta_outL, theta_outR, distance, isintersect

def readFrameRight(frame):
	rho_out, theta_out, theta_outL,theta_outR, L_dist, isintersect = readFrame(frame, False)
	# cv2.namedWindow('image Mask')
	cv2.namedWindow('frame')
	# print "Intersection Detection:", isintersect

	imgini = frame

	imgSmall = cv2.resize(imgini,(150,150),cv2.INTER_AREA)
	img=cv2.cvtColor(imgSmall,cv2.COLOR_BGR2HSV)

	# lowboundsYellow = np.array([25,29,60])
	# upboundsYellow = np.array([28,236,255])

	# lbPurp = np.array([141,18,97])
	# ubPurp = np.array([170,112,143])

	# maskY = cv2.inRange(display, lowboundsYellow, upboundsYellow)
	# maskP = cv2.inRange(display, lbPurp, ubPurp)
	# maskTot = cv2.bitwise_or(maskY,maskP)
	maskB = cv2.inRange(img,np.array([7,61,58]),np.array([115,222,255]))
	# colLookAheadIDs = np.where(np.equal(maskTot[:,lookAheadColumn],255))
	# check1 = np.any(maskB[137,130:]==255)
	# check2 = np.any(maskB[133,130:]==255)
	# check3 = np.any(maskB[139,130:]==255)

	checkLOC = maskB[112,141] == 255
	checkLOC2 = maskB[118,145] == 255
	checkLOC3 = maskB[115,142] == 255
	# cv2.line(imgSmall,(140,0),(140,149),[255,255,0],1)
	# cv2.line(imgSmall,(0,120),(149,120),[255,255,0],1)
	# cv2.imshow('frame',imgSmall)
	# # cv2.imshow('image Mask',maskB)
	# cv2.waitKey()
	print "R Lane Detection:", checkLOC, checkLOC2, checkLOC3
	if checkLOC or checkLOC2 or checkLOC3:
		detectR = True
	else:
		detectR = False
	# print "Right Lane Detection:", check1, check2, check3
	# # check2 = np.any(maskB[0,75:]==255)
	# # works on the assumption that the camera will not see both the intersecting line and far line of the road
	# print "RLane Sum:" , sum([check1,check2,check3])
	# if sum([check1,check2,check3])<=1:
	# 	detectR = False
	# else:
	# 	detectR = True

	return rho_out, theta_out, theta_outL, theta_outR, L_dist, detectR

def UpdateMotor(newDC):
	# speedLimit = 7.9 #Comp speed limit
	speedLimit = 7.95
	# Reverse ~ 7.14%
	# Neurtral ~ 7.5%
	# Forward ~ 7.8%
	if newDC > speedLimit:
		newDC = speedLimit
	dp.ChangeDutyCycle(newDC)
	time.sleep(0.5)

def UpdateSteering(neutral, controlSign):
	# 9% is Left
	# 13% is Right
	if neutral - controlSign <= 9:
		newDC = 9
	elif neutral - controlSign >= 13:
		newDC = 13
	else:
		newDC = neutral - controlSign
	sp.ChangeDutyCycle(newDC)
	# print "New Steering DC:" ,newDC
	time.sleep(0.5)

def lookStopLight():
	out = []
	i = 0
	while i < 4:
		word = bus.read_word_data(DEVICE_ADDRESS, 0)
		if word == 0xaa55:
			i += 1
			word = bus.read_word_data(DEVICE_ADDRESS, 1)
			if word == 0xaa56 or word == 0xaa55:
				checksum = bus.read_word_data(DEVICE_ADDRESS, 2)
				signature = bus.read_word_data(DEVICE_ADDRESS, 3)
				xctr = bus.read_word_data(DEVICE_ADDRESS, 4)
				yctr = bus.read_word_data(DEVICE_ADDRESS, 5)
				width = bus.read_word_data(DEVICE_ADDRESS, 6)
				height = bus.read_word_data(DEVICE_ADDRESS, 7)
				out.append(float(signature))
	return out

def StopCar():
	UpdateMotor(7.4)
	time.sleep(0.5)

# Initialize Robot and Passenger Locations:
# outpath, outcommands = UpdateMap(1)
outcommands = ['R','S1']
#outcommands = ['R']
outpath = [13, 14, 18, 20, 5, 8, 11, 12]
curState = outcommands.pop()
curLoc = outpath.pop()
run_robot = True
is_pickup = False
move_complete = False
isintersect = False


# initialize the camera and grab a reference to the raw camera capture
# global camera
# global rawCapture
global DEVICE_ADDRESS
global isintersect
global dp
global sp
global radio
global vs

# camera = PiCamera()
# camera.resolution = (1280, 720)
# camera.framerate = 60
# rawCapture = PiRGBArray(camera, size=(1280, 720))
vs = PiVideoStream().start()
radio = radioSetup()

# startup Pixy Camera I2C Comm
# bus = smbus.SMBus(1)
# DEVICE_ADDRESS = 0x54

# allow the camera to warmup
time.sleep(0.2)


# Initialize the pins
steerPin = 32
drivePin = 33
GPIO.setmode(GPIO.BOARD)
GPIO.setup(drivePin,GPIO.OUT)
GPIO.setup(steerPin,GPIO.OUT)
dp = GPIO.PWM(drivePin,50)
sp = GPIO.PWM(steerPin,50)

print "Starting Motor"
dp.start(7.5)
time.sleep(3)
sp.start(10.8)



	# Control your movement:
while run_robot:
	print "Current State:", curState
	if 'S' in curState:
		# go straight until you need to do something else
		toskip = int(curState[1])
		nxturn = outcommands[-1]
		GoStraight(toskip,nxturn)
		move_complete = True

	elif curState == 'R':
		# turn right until do something else
		TurnRight()
		move_complete = True

	if move_complete:
		if outcommands:
			curState = outcommands.pop()
		if outpath:
			curLoc = outpath.pop()
			move_complete = False
		if not outcommands and not outpath:
			run_robot = False

	# if curLoc == 13 and not is_pickup:
		# PsngerPickup()
		# is_pickup == True
		# outpath, outcommands = UpdateMap(0)
		# curState = outcommands.pop()
		# curLoc = outpath.pop()
	# elif curLoc == 13 and is_pickup:
		# PsngerDropoff()
		# run_robot = False
		# break
