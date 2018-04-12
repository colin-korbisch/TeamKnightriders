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



def GoStraight(skipIntersect):
	# continue going straight until you skip the correct num intersections
	# once you do, approach the intersection until you are at the correct distance
	# once at the correct distance, leave this state to begin your turn



def PsngerPickup():




# Initialize Robot and Passenger Locations:
outpath, outcommands = UpdateMap(1)
curState = outcommands.pop()
curLoc = outpath.pop()
run_robot = True
is_pickup = False


# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 360)
camera.framerate = 60
rawCapture = PiRGBArray(camera, size=(640, 360))
 
# allow the camera to warmup
time.sleep(0.1)


for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# Do some image processing stuff...


	# Control your movement:

	if 'S' in curState:
		# go straight until you need to do something else
		intersectToSkip = curState[1]
		GoStraight(intersectToSkip)
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