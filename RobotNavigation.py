from BreadthSearch import *
from PiSkyPixy import *
import RPi.GPIO as GPIO
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

def pixel2coord(pixelLoc):
	# stuff
	# need pixel to block/position conversion
	return pixelPosition

def TurnLeft():

def TurnRight():
	sendState(1)
	UpdateMotor(2)
	isTurning = False
	lineSkip = True
	checkComplete = False

	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		
		# Do some image processing stuff...

		rho_out, theta_out, distancePx = readFrameRight(frame,True)
		if not isTurning:
			if distancePx >= 20: # How far the first intersection line travels down the frame BEFORE the next intersection line appears in frame
				lineSkip = False

			if distancePx >= 10 and not lineSkip# Adjust this constant as needed
				UpdateSteering(13)
				isTurning = True

		if not isinstance(theta_out,np.ndarray):
			theta_out = np.array(theta_out)
		if not isinstance(rho_out,np.ndarray):
			rho_out = np.array(rho_out)		

		#Find right lines:
		arr1 = np.greater_equal(theta_out,np.pi/2)
		arr2 = np.lesser_equal(theta_out,3*np.pi/4)
		and_arr = np.logical_and(arr1,arr2)
		rightLineIDs = np.where(and_arr)

		#Find left lines:
		arr1 = np.greater_equal(theta_out,np.pi/4)
		arr2 = np.lesser_equal(theta_out,np.pi/2)
		and_arr = np.logical_and(arr1,arr2)
		leftLineIDs = np.where(and_arr)

		if isTurning:
			# Continue turn until a line around 45deg crosses the frame
			# Now you need to check the frame until you complete the turn

			# Find 45deg lines
			arr1 = np.greater_equal(theta_out,3*np.pi/16)
			arr2 = np.lesser_equal(theta_out, 5*np.pi/16)
			and_arr = np.logical_and(arr1,arr2)
			diagLineIDs = np.where(and_arr)
			if diagLineIDs[0].size:
				checkComplete = True

		if (leftLineIDs[0].size or rightLineIDs[0].size) and checkComplete
			isTurning = False
			UpdateSteering(11)
			break


def UpdateMap(pickup):
	# This has been commented out for the dry-run around the perimeter
	rad_Data = []
	robotLoc = []
	psngrLoc = []
	# if pickup:
		# while not robotLoc and not psngrLoc:
		# 	rad_Data = getRadioSignal()
		# 	if "ROBOT" in rad_Data:
		# 		robotLoc = rad_Data
		# 	elif "PSGR" in rad_Data:
		# 		psngrLoc = rad_Data
		# robPos, psngrPos = pixel2coord(robotLoc, psngerLoc)
		# outpath, outcommands = cityPath(robPos[0], robPos[1], psngrPos[0], psngrPos[1])
	# else:
		# while not robotLoc and not dropLoc:
		# 	rad_Data = getRadioSignal()
		# 	if "ROBOT" in rad_Data:
		# 		robotLoc = rad_Data
		# 	elif "DROP" in rad_Data:
		# 		dropLoc = rad_Data
		# robPos, psngrPos = pixel2coord(robotLoc, dropLoc)
		# outpath, outcommands = cityPath(robPos[0], robPos[1], dropPos[0], dropPos[1])
	outpath = [12, 0, 3, 6, 9, 10, 11, 8, 5, 2, 13]
	outcommands = ['S0', 'R', 'S1', 'R', 'S0', 'R', 'S1', 'R']
	return outpath, outcommands

def PollCurLoc():
	robotLoc = []
	while not robotLoc:
		rad_Data = getRadioSignal()
		data = rad_Data.split(" ")
		data
	robNode = pixel2node(robotLoc)

def sendState(stateIndex):
	#send the stateID to Arduino

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
	# once you do, approach the intersection until you are at the correct distance
	# once at the correct distance, leave this state to begin your turn


	UpdateMotor(12)
	leave = False
	numInt = 0
	isintersect = False
	oldInt = False

	setpointL = 0.1571 # 9 Degrees
	setpointR = 2.7925 # 160 Degrees

	if nxTurn == 'L':
		dist = #Ldistance
	else:
		dist = 100 # right distance



	sendState(0)
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		# Do some image processing stuff...
		oldInt = isintersect
		rho_out, theta_out, isintersect = readFrame(frame)

		# Counting out number of intersections
		if numInt != skipIntersect:
			if isintersect and not oldInt:
				enterInt = True
			if not isintersect and oldInt and enterInt:
				leaveInt = True
				enterInt = False
				numInt = numInt + 1


		if numInt == skipIntersect:
			break

		if not isinstance(theta_out,np.ndarray):
			theta_out = np.array(theta_out)
		if not isinstance(rho_out,np.ndarray):
			rho_out = np.array(rho_out)		


		#Find right lines:
		arr1 = np.greater_equal(theta_out,np.pi/2)
		arr2 = np.lesser_equal(theta_out,3*np.pi/4)
		and_arr = np.logical_and(arr1,arr2)
		rightLineIDs = np.where(and_arr)

		# Right Error:
		if rightLineIDs[0].size:
			RError = setpointR - np.mean(theta_out[rightLineIDs])

		#Find left lines:
		arr1 = np.greater_equal(theta_out,np.pi/4)
		arr2 = np.lesser_equal(theta_out,np.pi/2)
		and_arr = np.logical_and(arr1,arr2)
		leftLineIDs = np.where(and_arr)

		# Left Error:
		if leftLineIDs[0].size:
			LError = setpointL - np.mean(theta_out[leftLineIDs])

		# Mean Error:
		if LError:
			if RError:
				ErrorSignal = (LError+RError)/2
			else:
				ErrorSignal = LError
		elif RError:
			ErrorSignal = RError
		else:
			ErrorSignal = 0

		#Find horz lines:
		rotateTheta = theta_out - np.pi
		arr1 = np.greater_equal(rotateTheta,7*np.pi/8)
		arr2 = np.lesser_equal(rotateTheta,np.pi/8)
		and_arr = np.logical_and(arr1,arr2)
		horzLineIDs = np.where(and_arr)

		# Check for stop lights
		# NEED TO TIGHTEN-UP THIS CONDITION
		if not isintersect:
			signature = lookStopLight()

			if sum(np.equal(signature,1)) > 3:
				StopCar()
		
def sendTurnError(error_signal)		

def PsngerPickup():

def readFrame(frame, isintersect):
	def interDetect(mask,row,isintersect):
		interLine = mask[row]
		findYellow = np.where(np.equal(interLine,255))
		if findYellow[0].size:
			findLine = np.amax(findYellow)
		else
			findLine = 0
		if findLine < 100:
			isintersect = True
		elif findLine > 100 and isintersect:
			interLineTop = mask[row-10]
			interLineBot = mask[row+10]
			fLineT = np.where(np.equal(interLineTop,255))
			fLineB = np.where(np.equal(interLineBot,255))
			if fLineT[0].size:
				findYelTop = np.amax(fLineT)
			else:
				findYelTop = 0
			if fLineB[0].size:
				findYelBot = np.amax(fLineB)
			else:
				findYelBot = 0
			if findYelTop < 100 and findYelBot < 100:
				isintersect = True
			else:
				isintersect = False
		return isintersect

	rho_out = []
	theta_out = []
	intersectDetectRow = 20 # need to test this constant

	imgSmall = cv2.resize(imgini,(150,150),cv2.INTER_AREA)
	img=cv2.cvtColor(imgSmall,cv2.COLOR_BGR2HSV)
	
	# lowboundsYellow = np.array([10,59,67])
	# upboundsYellow = np.array([39,229,219])

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

	lines = cv2.HoughLines(edges,0.5,np.pi/180,39)

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
	l1 = interDetect(maskTot,intersectDetectRow-1,isintersect)
	l2 = interDetect(maskTot,intersectDetectRow,isintersect)
	l3 = interDetect(maskTot,intersectDetectRow+1,isintersect)

	if sum([l1,l2,l3]) > 1:
		isintersect = True
	elif sum([l1,l2,l3]) < 1 and isintersect:
		isintersect = False


	return rho_out, theta_out, isintersect

def readFrameRight(frame):
	def interDetect(mask,row,isintersect):
		interLine = mask[row]
		findYellow = np.where(np.equal(interLine,255))
		if findYellow[0].size:
			findLine = np.amax(findYellow)
		else
			findLine = 0
		if findLine < 100:
			isintersect = True
		elif findLine > 100 and isintersect:
			interLineTop = mask[row-10]
			interLineBot = mask[row+10]
			fLineT = np.where(np.equal(interLineTop,255))
			fLineB = np.where(np.equal(interLineBot,255))
			if fLineT[0].size:
				findYelTop = np.amax(fLineT)
			else:
				findYelTop = 0
			if fLineB[0].size:
				findYelBot = np.amax(fLineB)
			else:
				findYelBot = 0
			if findYelTop < 100 and findYelBot < 100:
				isintersect = True
			else:
				isintersect = False
		return isintersect

	rho_out = []
	theta_out = []
	lookAheadColumn = 140 #Another constant to adjust as needed

	imgSmall = cv2.resize(imgini,(150,150),cv2.INTER_AREA)
	img=cv2.cvtColor(imgSmall,cv2.COLOR_BGR2HSV)
	
	# lowboundsYellow = np.array([10,59,67])
	# upboundsYellow = np.array([39,229,219])

	lowboundsYellow = np.array([25,29,60])
	upboundsYellow = np.array([28,236,255])

	lbPurp = np.array([141,18,97])
	ubPurp = np.array([170,112,143])

	maskY = cv2.inRange(display, lowboundsYellow, upboundsYellow)
	maskP = cv2.inRange(display, lbPurp, ubPurp)
	maskTot = cv2.bitwise_or(maskY,maskP)

	colLookAheadIDs = np.where(np.equal(maskTot[:,lookAheadColumn],255))

	# works on the assumption that the camera will not see both the intersecting line and far line of the road
	if colLookAheadIDs[0].size:
		distance = min(colLookAheadIDs)
	else:
		distance = 150

	res = cv2.bitwise_and(imgSmall,imgSmall, mask=maskTot)
	gray = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)
	edges = cv2.Canny(gray,50,150,apertureSize = 3)
	lines = cv2.HoughLines(edges,0.5,np.pi/180,39)

	for line in lines:
		line = line[0]
		rho = line[0]
		theta = line[1]
		rho_out.append(rho)
		theta_out.append(theta)

	return rho_out, theta_out, distance

def UpdateMotor(newDC):
	dp.ChangeDutyCycle(newDC)

def UpdateSteering(newDC):
	# 9% is Left
	# 13% is Right
	sp.ChangeDutyCycle(newDC)

# Initialize Robot and Passenger Locations:
outpath, outcommands = UpdateMap(1)
curState = outcommands.pop()
curLoc = outpath.pop()
run_robot = True
is_pickup = False
move_complete = False
isintersect = False


# initialize the camera and grab a reference to the raw camera capture
global camera
global rawCapture
global DEVICE_ADDRESS
global isintersect
global dp
global sp

camera = PiCamera()
camera.resolution = (640, 360)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 360))

# startup Pixy Camera I2C Comm
bus = smbus.SMBus(1)
DEVICE_ADDRESS = 0x54

# allow the camera to warmup
time.sleep(0.1)


# Initialize the pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(drivePin,GPIO.OUT)
GPIO.setup(steerPin,GPIO.OUT)
steerPin = 32
drivePin = 33
dp = GPIO.PWM(drivePin,50)
sp = GPIO.PWM(steerPin,50)

dp.start(0)
sp.start(11)



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


