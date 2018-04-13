from BreadthSearch import *
from PiSkyPixy import *
import RPi.GPIO
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

def pixel2coord(pixelLoc):
	# stuff
	# need pixel to block/position conversion
	return pixelPosition

def sendStateCommand(stateIndex):
	# more stuff

def TurnLeft():

def TurnRight():

def UpdateMap(pickup):
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
		if "ROBOT" in rad_Data:
			robotLoc = rad_Data
	robNode = pixel2node(robotLoc)

def sendState(stateIndex):
	#send the stateID to Arduino

def lookStopLight():
	out = []
	i = 0
	while i < 6
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
	leave = False
	numDownIntersect = False

	setpointL = np.pi/2 - 0.1
	setpointR = np.pi/2 + 0.1

	if nxTurn == 'L':
		dist = #Ldistance
	else:
		dist = #Rdistance

	leaveIntersection = #leaving the intersection distance

	sendState(0)
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		# Do some image processing stuff...
		rho_out, theta_out = readFrame(frame)
		signature = lookStopLight()

		if sum(np.equal(signature,1)) > 3:
			return True
			break
		leftError = []
		rightError = []
		for i in range(len(theta_out)):
			theta = theta_out[i]
			if abs(theta) <= 0.1 or theta >= 2*np.pi-0.1:
				theta = np.pi

			if abs(theta) <= np.pi + 0.1 and abs(theta) >= np.pi - 0.1:
				yloc = np.sin(theta) * rho_out[i]
				if yloc >= dist and skipIntersect == 0:
					leave = True
					break
				elif yloc >= leaveIntersection:
					numDownIntersect = True

			elif abs(theta) <= np.pi/2 and abs(theta) >= np.pi/4:
				leftError.append(abs(setpointL - abs(theta)))

			elif abs(theta) >= np.pi/2 and abs(theta) <= 3*np.pi/4:
				rightError.append(abs(setpointR - abs(theta)))
		if leave:
			break
		if leftError:
			lE = np.mean(leftError)
		if rightError:
			rE = np.mean(rightError)
		if lE and rE:
			Esignal = np.mean([lE,rE])
		elif lE:
			Esignal = lE
		elif rE:
			Esignal = rE
		if Esignal:
			sendTurnError(Esignal)			
		if numDownIntersect:
			skipIntersect -= 1
			numDownIntersect = False


		


def PsngerPickup():


def readFrame(frame):
	rho_out = []
	theta_out = []

	imgSmall = cv2.resize(imgini,(150,150),cv2.INTER_AREA)
	img=cv2.cvtColor(imgSmall,cv2.COLOR_BGR2HSV)
	
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
		rho_out.append(rho)
		theta_out.append(theta)

	return rho_out, theta_out

# Initialize Robot and Passenger Locations:
outpath, outcommands = UpdateMap(1)
curState = outcommands.pop()
curLoc = outpath.pop()
run_robot = True
is_pickup = False


# initialize the camera and grab a reference to the raw camera capture
global camera
global rawCapture
global DEVICE_ADDRESS

camera = PiCamera()
camera.resolution = (640, 360)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 360))

# startup Pixy Camera I2C Comm
bus = smbus.SMBus(1)
DEVICE_ADDRESS = 0x54

# allow the camera to warmup
time.sleep(0.1)




	# Control your movement:
while run_robot:
	if 'S' in curState:
		# go straight until you need to do something else
		toskip = curState[1]
		nxturn = outcommands[-1]
		bstop = GoStraight(toskip,nxturn)
		if bstop:
			StopCar()

	elif curState == 'R':
		# turn right until do something else
		TurnRight()
	elif curState == 'L':
		# turn left until something else
		TurnLeft()
	if move_complete:
		curState = outcommands.pop()
		curLoc = outpath.pop()

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


while run_robot:
	if curState == 'S':
		GoStraight()
	elif curState == 'R':
		TurnRight()
	elif curState == 'L':
		TurnLeft()
	PsngerPickup()
	# After pickup is completed, build new path to the drop-off area
	outpath, outcommands = UpdateMap(0)
	PsngerDropoff()