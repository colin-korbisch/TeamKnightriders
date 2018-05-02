from BreadthSearch import *
from PiSkyPixy import *
from PedestrianDetectionv2 import *
import RPi.GPIO as GPIO
from picamera.array import PiRGBArray
from picamera import PiCamera
from imutils.video.pivideostream import PiVideoStream
import imutils
import time
import cv2
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(FALSE)

def CheckPedestrian():
    TrigLeft = 11 
	EchoLeft = 13
	TrigRight = 15
	EchoRight = 16
	#ThresholdValue = 

	GPIO.setup(TrigLeft,GPIO.OUT)
	GPIO.setup(TrigRight,GPIO.OUT)
	GPIO.setup(EchoLeft,GPIO.IN)
	GPIO.setup(EchoRight,GPIO.IN)

	GPIO.output(TrigLeft, False) #wait for left sensor to settle
	time.sleep(2)
	GPIO.output(TrigRight, False) #wait for right sensor to settle
	time.sleep(2)

	while (1):
		GPIO.output(TrigLeft, True)  #give 10uS pulse to trigger the module 
		time.sleep(0.00001)
		GPIO.output(TrigLeft, False) #go back to low
			
		while GPIO.input(EchoLeft)==0:
			Left_Pulse_Start = time.time()

		while GPIO.input(EchoLeft)==1:
			Left_Pulse_End = time.time()

		Left_Pulse_Duration = Left_Pulse_End - Left_Pulse_Start
		LeftDistance = Left_Pulse_Duration * 17150

		GPIO.output(TrigRight, True)  #give 10uS pulse to trigger the module 
		time.sleep(0.00001)
		GPIO.output(TrigRight, False) #go back to low

		while GPIO.input(EchoRight)==0:
			Right_Pulse_Start = time.time()

		while GPIO.input(EchoRight)==1:
			Right_Pulse_End = time.time()

		Right_Pulse_Duration = Right_Pulse_End - Right_Pulse_Start
		RightDistance = Right_Pulse_Duration * 17150

		LeftDistance = round(LeftDistance,2)
		RightDistance = round(RightDistance,2)

		#print "left:", LeftDistance
		#print "right:", RightDistance
		
		if(LeftDistance<2 or RightDistance<2):
			StopCar()

		time.sleep(0.1)

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
	setLDist =  45


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
			UpdateMotor(7.85)
		return LError, LDistE
def StraightLineControl(EsigHistory,LaneDist):
	# Define PID control constants:
	# Angle PID
	kp = 0.25
	kd = 3.2
	ki = 0.001

	# Lane Distance Controller:
	lkp = 0.053
	lkd = 0.0019
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

	# Weighting combination of two PID controllers:
	# 1: Angular orientation of the two lane lines (Angle PID)
	# 2: Horiztional distance of the left lane edge from the left edge of robot's visual frame (Lane Distance PID)
	steerContro = 0.55*(kp*EsigHistory[-1] + kd*Edif + ki*Esum) + 0.45*(lkp*LaneDist[-1] + lkd*Ldif + lki*Lsum)

	UpdateSteering(10.8,steerContro)

	# Slowing down the motor proportionally to the error signal
	if 8.2-0.05*abs(LaneDist[-1]) <= 7.88:
		UpdateMotor(7.5)
		time.sleep(0.2)
		UpdateMotor(7.88)
	else:
		UpdateMotor(8.2-0.05*abs(LaneDist[-1]))

def pixel2coord(pixelLoc):
	xLoc = pixelLoc[0]
	yLoc = pixelLoc[1]

	if xLoc >=240:
		if xLoc > 250 and yLoc > 100:
			blkPos = [1,4]
		elif yLoc > 135:
			blkPos = [1,1]
		elif xLoc <= 250 and yLoc > 130:
			blkPos = [1,2]
		elif yLoc <= 135:
			blkPos = [1,3]

		elif xLoc > 250 and yLoc < 100:
			blkPos = [4,4]
		elif yLoc < 30:
			blkPos = [4,3]
		elif xLoc < 250 and yLoc < 100:
			blkPos = [4,2]
		elif yLoc >=40 and yLoc < 100:
			blkPos = [4,1]

	elif xLoc >=170 and xLoc <=230:
		if yLoc > 100:
			if yLoc > 150:
				blkPos = [2,1]
			elif xLoc < 185:
				blkPos = [2,2]
			elif yLoc < 150:
				blkPos = [2,3]
			else:
				blkPos = [2,4]
		else:
			if yLoc > 30:
				blkPos = [5,1]
			elif xLoc < 185:
				blkPos = [5,2]
			elif yLoc < 30:
				blkPos = [5,3]
			else:
				blkPos = [5,4]
	else:
		if yLoc > 100:
			if yLoc > 150:
				blkPos = [3,1]
			elif xLoc < 50:
				blkPos = [3,2]
			elif yLoc < 150:
				blkPos = [3,3]
			else:
				blkPos = [3,4]
		else:
			if yLoc > 30:
				blkPos = [6,1]
			elif xLoc < 50:
				blkPos = [6,2]
			elif yLoc < 30:
				blkPos = [6,3]
			else:
				blkPos = [6,4]
	return blkPos

def TurnLeft():
	# Continue going straight until the next intersection
	# Turn left when the horizontal tape is some set distance away from the robot
	print "Left Turn State"
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

		rho_out, theta_out, theta_outL, theta_outR, L_dist, centDist = readFrameLeft(frame)
		# print "Can we see the right lane?: ", detectR
		# rawCapture.truncate(0)
		if not isTurning:
			if centDist>5: #distance to tune for left turning. the larger the value, the later the car will turn
				print "Now Turning Left:"
				UpdateSteering(9,0)
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
			arr1 = np.greater_equal(theta_outR,2*np.pi/16)
			arr2 = np.less_equal(theta_outR, 6*np.pi/16)
			and_arr = np.logical_and(arr1,arr2)
			diagLineIDs = np.where(and_arr)
			if diagLineIDs[0].size:
				checkComplete = True

		if checkComplete:
			isTurning = False
			UpdateSteering(11,0)
			print "Turn Complete"
			break

def TurnRight():
	# Continue going straight until you hit the next intersection
	# Turn right once the right lane tape is detected in region of visual frame
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
			arr1 = np.greater_equal(theta_outL,2*np.pi/16)
			arr2 = np.less_equal(theta_outL, 6*np.pi/16)
			and_arr = np.logical_and(arr1,arr2)
			diagLineIDs = np.where(and_arr)
			if diagLineIDs[0].size:
				checkComplete = True

		if checkComplete:
			isTurning = False
			UpdateSteering(11,0)
			print "Turn Complete"
			break

def UpdateMap(pickup):	

	robotLoc = []
	psngrLoc = []
	destLoc = []
	robL, psngrL, destL = getRadioSignal(radio)
	robData = split(robL)
	robotLocation = [robData[1], robData[2]]
	robPos = pixel2coord(robotLocation)

	# Generating path commands to go from robot's initial position to the passenger in order to pickup
	if pickup:
		psngrL = split(psngrL)
		psngrLocation = [psngrL[1], psngrL[2]]
		psngrPos = pixel2coord(psngrLocation)
		outpath, outcommands = cityPath(robPos[0], robPos[1], psngrPos[0], psngrPos[1])
		i = -1
		newcomm = []
		while i >= -1*len(outcommands):
			if outcommands[i] != 'S':
				newcomm.append(outcommands[i])
				i = i-1
			else:
				goS = 0
				while outcommands[i-1] == 'S':
					goS = goS + 1
					i = i-1
				i = i-1
				newcomm.append('S{}'.format(goS))
		outcommands = newcomm[::-1]
	# Generating path commands to go from robot position (at pickup) to the passenger's destination in order to drop-off
	else:
		destL = split(destL)
		destLocation = [destL[1], destL[2]]
		destPos = pixel2coord(destLocation)
		# robPos, psngrPos = pixel2coord(robotLoc, dropLoc)
		outpath, outcommands = cityPath(robPos[0], robPos[1], destPos[0], destPos[1])
		i = -1
		newcomm = []
		while i >= -1*len(outcommands):
			if outcommands[i] != 'S':
				newcomm.append(outcommands[i])
				i = i-1
			else:
				goS = 0
				while outcommands[i-1] == 'S':
					goS = goS + 1
					i = i-1
				i = i-1
				newcomm.append('S{}'.format(goS))
		outcommands = newcomm[::-1]
	# outpath = [12, 0, 3, 6, 9, 10, 11, 8, 5, 2, 13]
	# outcommands = ['S0', 'R', 'S1', 'R', 'S0', 'R', 'S1', 'R']
	return outpath, outcommands

def sendState(stateIndex):
	#send the stateID to Arduino
	#when robot position = passenger position (with offset), send pixie pin high, wait for pixie to respond/toggle motor, send arm pin high


def lookStopLight():
	out = []
	i = 0
	while i < 4
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

def GoStraight(skipIntersect,nxTurn):
	# continue going straight until you skip the correct num intersections
	EsigHistory = []
	LdistHist = []
	EsignalM = 0
	LsigM = 0
	
	startTime = time.time()
	# print "going straight"
	UpdateMotor(7.88)
	leave = False
	numInt = 0
	isintersect = False
	enterInt = False
	leaveInt = True
	oldInt = False

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
		# print "Numb of Inter Passed:", numInt
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
				outpath.pop()
			# print "Did Enter Int?", enterInt
			# print "Did Leave Int?", leaveInt
		else:
			# Once you have skipped the requisit number of intersections, leave "GoStraight" mode
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
			UpdateMotor(7.8)
			time.sleep(0.1)

		#Find horz lines:
		# arr1 = np.greater_equal(rotateTheta,7*np.pi/8)
		# arr2 = np.less_equal(rotateTheta,np.pi/8)
		# and_arr = np.logical_and(arr1,arr2)
		# horzLineIDs = np.where(and_arr)
		
def sendTurnError(error_signal)		

def PsngerPickup():
	#Pixie pin 36 to 12
	GPIO.setup(36, GPIO.OUT) 
	#Arm pin 37 to 11
	GPIO.setup(37, GPIO.OUT)

	#If we are at passenger location turn pixie
	GPIO.output(36, 1)
	time.sleep(.1)
	GPIO.output(36,0)
	time.sleep(1)
	
	#If pixie sends signal that center of mass of passenger is okay/if the GPS offset is okay
	#pick up passenger
	GPIO.output(37,1)
	time.sleep(.1)
	GPIO.output(37,0)
	time.sleep(3)

def PsngerDropoff():
	#GPS offset okay, initiate drop off passenger
	#drop off passenger
	GPIO.output(37,1)
	time.sleep(.1)
	GPIO.output(37,0)
	time.sleep(3)


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

	lowboundsYellow = np.array([15,51,34])
	upboundsYellow = np.array([34,248,220])

	lbPurp = np.array([141,18,97])
	ubPurp = np.array([170,250,255])

	maskY = cv2.inRange(img, lowboundsYellow, upboundsYellow)
	maskP = cv2.inRange(img, lbPurp, ubPurp)
	maskTot = cv2.bitwise_or(maskY,maskP)
	# maskB = cv2.inRange(img,np.array([7,61,58]),np.array([115,222,255]))
	TopLine = maskTot[0,0:75]
	if np.any(TopLine==255):
		distance = np.max(np.where(TopLine==255))
	else:
		distance = 75

	res = cv2.bitwise_and(imgSmall,imgSmall, mask=maskTot)

	gray = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)

	edges = cv2.Canny(gray,50,150,apertureSize = 3)

	lines = cv2.HoughLines(edges,0.5,np.pi/180,35)
	# Left Line searches the left half of the frames (all rows, columns from 0 up to 70)
	leftLines = cv2.HoughLines(edges[:,0:70],0.5,np.pi/180,20) #Colin - looking at top and bottom of screen or left and right???
	# Colin: looking at the left half of the screen

	# Right Line searches the right half of hte frame (all rows, columns from 80 up to 149.)
	# Indexing starts at 0. A frame of width 150 starts at 0 and ends it's index at 149

	rightLines = cv2.HoughLines(edges[:,80:149],0.5,np.pi/180,20)
	# centerline = cv2.HoughLines(edges[:,]) #Tasha did this for the center of intersection to start left turn
	# See my updates in readFrameLeft (Colin)

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
	ar1 = np.where(maskB[130,:]==255)
	if ar1[0].size:
		l1 = np.max(ar1)
		if l1<75:
			checkTop = True
		else:
			checkTop = False
	else:
		checkTop = False

	l3 = np.max(np.where(maskTot[137,:]==255))
	if l3<75:
		checkMid = True
	else:
		checkMid = False
	l2 = np.max(np.where(maskTot[145,:]==255))
	if l2<75:
		checkBot = True
	else:
		checkBot = False

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

	lowboundsYellow = np.array([25,29,60])
	upboundsYellow = np.array([28,236,255])

	lbPurp = np.array([141,18,97])
	ubPurp = np.array([170,112,143])

	maskY = cv2.inRange(img, lowboundsYellow, upboundsYellow)
	maskP = cv2.inRange(img, lbPurp, ubPurp)
	maskTot = cv2.bitwise_or(maskY,maskP)
	# maskB = cv2.inRange(img,np.array([7,61,58]),np.array([115,222,255]))
	# colLookAheadIDs = np.where(np.equal(maskTot[:,lookAheadColumn],255))
	# check1 = np.any(maskB[137,130:]==255)
	# check2 = np.any(maskB[133,130:]==255)
	# check3 = np.any(maskB[139,130:]==255)

	# check within a small region of the frame whether the lane tape is there:
	circle = maskTot[115:120,141:146]


	# OLD METHOD:
	# checkLOC = maskTot[120,140] == 255
	# checkLOC2 = maskTot[120,145] == 255
	# checkLOC3 = maskTot[118,142] == 255
	# # cv2.line(imgSmall,(140,0),(140,149),[255,255,0],1)
	# # cv2.line(imgSmall,(0,120),(149,120),[255,255,0],1)
	# # cv2.imshow('frame',imgSmall)
	# # # cv2.imshow('image Mask',maskB)
	# # cv2.waitKey()
	# print "R Lane Detection:", checkLOC, checkLOC2, checkLOC3
	# if checkLOC or checkLOC2 or checkLOC3:

	# if the lane tape is in that region, then we can turn right
	if np.any(circle==255):
		detectR = True
	else:
		detectR = False

	return rho_out, theta_out, theta_outL, theta_outR, L_dist, detectR

def readFrameLeft(frame):
	rho_out, theta_out, theta_outL,theta_outR, L_dist, isintersect = readFrame(frame, False)
	
	imgini = frame

	imgSmall = cv2.resize(imgini,(150,150),cv2.INTER_AREA)
	img=cv2.cvtColor(imgSmall,cv2.COLOR_BGR2HSV)

	lowboundsYellow = np.array([25,29,60])
	upboundsYellow = np.array([28,236,255])

	lbPurp = np.array([141,18,97])
	ubPurp = np.array([170,112,143])

	maskY = cv2.inRange(img, lowboundsYellow, upboundsYellow)
	maskP = cv2.inRange(img, lbPurp, ubPurp)
	maskTot = cv2.bitwise_or(maskY,maskP)

	# Blue tape masking from practice in lab office
	# maskB = cv2.inRange(img,np.array([7,61,58]),np.array([115,222,255]))


	checkCenter = np.where(maskTot[:,99:149] == 255)

	# To the the center of the frame, you need to check all rows and the 74th column:
	# checkCenter = np.where(maskTot[:,74]==255)

	if checkCenter[0].size:
		centerdist = np.min(checkCenter)
	else:
		centerdist = 0
	

	return rho_out, theta_out, theta_outL, theta_outR, L_dist, centerdist
def UpdateMotor(newDC):
	# speedLimit = 7.9 #Comp speed limit
	speedLimit = 7.87 # This sets the robot's absolute limit in PWM sent to motor driver
	# Reverse ~ 7.14%
	# Neurtral ~ 7.5%
	# Forward ~ 7.8%
	if newDC > speedLimit:
		newDC = speedLimit
	dp.ChangeDutyCycle(newDC)
	time.sleep(0.1)

def UpdateSteering(neutral, controlSign):
	# 9% is Left
	# 13% is Right
	# Limit the PWM output to bounds for servo
	if neutral - controlSign <= 9:
		newDC = 9
	elif neutral - controlSign >= 13:
		newDC = 13
	else:
		newDC = neutral - controlSign
	sp.ChangeDutyCycle(newDC)
	# print "New Steering DC:" ,newDC
	time.sleep(0.5)

def StopCar():
	# Reduce PWM of the motor to neutral
	UpdateMotor(7.4)
	time.sleep(1)


global outpath
global outcommands
# Initialize Robot and Passenger Locations:
outpath, outcommands = UpdateMap(1)
curState = outcommands.pop()
curLoc = outpath.pop()
run_robot = True
is_pickup = False
move_complete = False
isintersect = False


# initialize the camera and grab a reference to the raw camera capture
global vs
global DEVICE_ADDRESS
global isintersect
global dp
global sp
global radio

vs = PiVideoStream().start()
radio = radioSetup()

# startup Pixy Camera I2C Comm
bus = smbus.SMBus(1)
DEVICE_ADDRESS = 0x54

# allow the camera to warmup
time.sleep(0.2)


# Initialize the pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(drivePin,GPIO.OUT)
GPIO.setup(steerPin,GPIO.OUT)
steerPin = 32
drivePin = 33
dp = GPIO.PWM(drivePin,50)
sp = GPIO.PWM(steerPin,50)

dp.start(7.5)
sp.start(11)
time.sleep(3)



	# Control your movement:
while run_robot:
	if 'S' in curState:
		# go straight until you need to do something else
		toskip = curState[1]
		nxturn = outcommands[-1]
		GoStraight(toskip,nxturn)
		move_complete = True

	elif curState == 'R':
		# turn right until do something else
		TurnRight()
		move_complete = True

	elif curState == 'L':
		# turn left until something else
		TurnLeft()
	if move_complete:
		curState = outcommands.pop()
		curLoc = outpath.pop()
		move_complete = False

	if curLoc == 13 and not is_pickup:
		PsngerPickup()
		is_pickup == True
		outpath, outcommands = UpdateMap(0)
		curState = outcommands.pop()
		curLoc = outpath.pop()
	elif curLoc == 13 and is_pickup:
		PsngerDropoff()
		run_robot = False
		break