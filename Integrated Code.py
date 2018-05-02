from BreadthSearch import *
from PiSkyPixy import *
import RPi.GPIO as GPIO
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(FALSE)


# global camera
# global rawCapture
# global DEVICE_ADDRESS
# global isintersect
# global dp
# global sp
# global radio
class ROBOT
	def __init__():
			# Robot State Constants

			# Radio Setup
	        self.radio = radioSetup()

	        # Pi Cam Setup
	        self.camera = PiCamera()
	        self.camera.resolution = (640, 360)
			self.camera.framerate = 30
			self.rawCapture = PiRGBArray(camera, size=(640, 360))

	        # Robot 'Go Straight' Control Constants
	        self.kp = 0.5
	        self.kd = 0.009
	        self.ki = 0.00007

	        # Pin Setup
	        GPIO.setmode(GPIO.BOARD)
			GPIO.setup(drivePin,GPIO.OUT)
			GPIO.setup(steerPin,GPIO.OUT)
			steerPin = 32
			drivePin = 33
			self.dp = GPIO.PWM(drivePin,50)
			self.sp = GPIO.PWM(steerPin,50)

			#Pixy I2C Setup
			self.bus = smbus.SMBus(1)
			self.DEVICE_ADDRESS = 0x54



# Robot Test Run

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

def GoStraight(skipIntersect,nxTurn):
	# continue going straight until you skip the correct num intersections
	# once you do, approach the intersection until you are at the correct distance
	# once at the correct distance, leave this state to begin your turn


	# Define PID control constants:
	kp = 0.5
	kd = 0.009
	ki = 0.00007

	# Define error terms
	oldE = 0
	Esum = 0

	UpdateMotor(8)
	leave = False
	numInt = 0
	isintersect = False
	oldInt = False

	setpointL = 0.1571 # 9 Degrees
	setpointR = 2.7925 # 160 Degrees

	# if nxTurn == 'L':
	# 	dist = #Ldistance
	# else:
	# 	dist = 100 # right distance

	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		# Do some image processing stuff...
		oldInt = isintersect
		rho_out, theta_out, isintersect = readFrame(frame)

		# Check for stop lights
		# NEED TO TIGHTEN-UP THIS CONDITION
		if not isintersect:
			signature = lookStopLight()

			if sum(np.equal(signature,1)) > 3:
				StopCar()
				is_stopped = True
			else:
				UpdateMotor(8)
				is_stopped = False
		else:
			UpdateMotor(8)
			is_stopped = False

		# Counting out number of intersections
		if numInt != skipIntersect:
			if isintersect and not oldInt:
				enterInt = True
			if not isintersect and oldInt and enterInt:
				leaveInt = True
				enterInt = False
				numInt = numInt + 1
		else:
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

		Esum = Esum + ErrorSignal
		if not oldE:
			Edif = 0
		else:
			Edif = ErrorSignal - oldE
			oldE = ErrorSignal

		steerContro = kp*oldE + kd*Edif + ki*Esum

		UpdateSteering(11+steerContro)

		#Find horz lines:
		rotateTheta = theta_out - np.pi
		arr1 = np.greater_equal(rotateTheta,7*np.pi/8)
		arr2 = np.lesser_equal(rotateTheta,np.pi/8)
		and_arr = np.logical_and(arr1,arr2)
		horzLineIDs = np.where(and_arr)



def TurnRight():
	# Define PID control constants:
	kp = 0.5
	kd = 0.009
	ki = 0.00007

	# Define error terms
	oldE = 0
	Esum = 0

	setpointL = 0.1571 # 9 Degrees
	setpointR = 2.7925 # 160 Degrees

	UpdateMotor(7.7)
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


		# Right Error:
		if rightLineIDs[0].size:
			RError = setpointR - np.mean(theta_out[rightLineIDs])

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

		Esum = Esum + ErrorSignal
		if not oldE:
			Edif = 0
		else:
			Edif = ErrorSignal - oldE
			oldE = ErrorSignal

		steerContro = kp*oldE + kd*Edif + ki*Esum
		if not isTurning:
			UpdateSteering(11+steerContro)

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
	# Reverse ~ 6%
	# Neurtral ~ 7.5%
	# Forward ~ 9%
	dp.ChangeDutyCycle(newDC)

def UpdateSteering(newDC):
	# 9% is Left
	# 13% is Right
	sp.ChangeDutyCycle(newDC)

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

def StopCar():
	UpdateMotor(5)

# Initialize Robot and Passenger Locations:
# outpath, outcommands = UpdateMap(1)
outcommands = ['R','S1']
outpath = [13, 1, 4, 12]
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
global radio

camera = PiCamera()
camera.resolution = (640, 360)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 360))
radio = radioSetup()

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
