# from BreadthSearch import *
# from PiSkyPixy import *
import RPi.GPIO as GPIO
# from picamera.array import PiRGBArray
# from picamera import PiCamera
import time
# import cv2

# Initialize the pins
steerPin = 32
drivePin = 33
GPIO.setmode(GPIO.BOARD)
GPIO.setup(drivePin,GPIO.OUT)
GPIO.setup(steerPin,GPIO.OUT)
dp = GPIO.PWM(drivePin,50)
sp = GPIO.PWM(steerPin,50)

dp.start(7.5)
sp.start(10.8)

for i in range(100):
	print "starting neutral"
	sp.ChangeDutyCycle(10.8)
	time.sleep(1)
	print "turn left"
	sp.ChangeDutyCycle(9)
	time.sleep(1)
	print "turn right"
	sp.ChangeDutyCycle(13)
	time.sleep(1)
quit()


# while True:
# 	mdc = mdc+0.01
# 	dp.ChangeDutyCycle(mdc)
# 	print mdc
# 	time.sleep(0.5)